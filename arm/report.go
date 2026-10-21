package arm

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
	"net"
	"strconv"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
)

// The controller pushes a status packet on each of its report ports to every
// connected client; nothing is sent to it. Port 30002 ("rich") carries the
// most fields, including configuration such as the TCP payload that the
// command socket on port 502 has no register to read.
const (
	defaultReportPort = 30002
	// reportStaleAfter bounds how old the latest packet may be before readers
	// stop trusting it, and how long a connection may go silent before it is
	// dropped. The controller pushes several packets a second, so a gap this
	// long means the stream is down.
	reportStaleAfter   = 2 * time.Second
	reportDialTimeout  = 3 * time.Second
	reportRetryInitial = 500 * time.Millisecond
	reportRetryCeiling = 10 * time.Second
	reportFirstTimeout = 3 * time.Second
	reportPollInterval = 50 * time.Millisecond
)

// Byte offsets into a rich report packet, counted from the start of the
// 4-byte length prefix. They match the xArm Python SDK's report parser
// (xarm/x3/base.py, __handle_report_normal, which the rich parser runs first).
const (
	reportLengthSize       = 4
	reportOffsetStateMode  = 4   // low nibble state, high nibble mode
	reportOffsetTCPLoad    = 115 // mass kg, then center of gravity x, y, z mm; 4 x float32
	reportOffsetCollisSens = 131
	reportOffsetTeachSens  = 132
	reportMinLength        = 133
	// Rich packets are about 600 bytes on current firmware; anything far larger
	// means the stream is not framed where we think it is.
	reportMaxLength = 4096
)

var errNoReport = errors.New("no controller report received yet")

// reportedTCPLoad is the payload the controller is currently compensating for,
// whichever source set it: UFACTORY Studio, the saved controller config, or a
// SetLoadParam call.
type reportedTCPLoad struct {
	MassKg float64
	// CenterOfGravityMM is relative to the flange in the tool frame. Firmware
	// older than 0.2.1 reported metres; no arm this module supports runs it.
	CenterOfGravityMM r3.Vector
}

// controllerReport is the part of a rich report packet the module decodes.
type controllerReport struct {
	State   byte
	Mode    byte
	TCPLoad reportedTCPLoad
}

// reportStream keeps a connection to a controller report port open in a
// background goroutine and holds the most recent packet.
//
// To expose another report field, decode it in decodeReport and add a reader
// that goes through latestReport, like tcpLoad. Readers never touch the
// network, so they are cheap to call from any code path.
type reportStream struct {
	addr   string
	logger logging.Logger

	// staleAfter and the retry bounds are fields so tests can shrink them.
	staleAfter                 time.Duration
	retryInitial, retryCeiling time.Duration

	mu       sync.RWMutex
	latest   controllerReport
	received time.Time // zero until the first packet

	cancel context.CancelFunc
	done   chan struct{} // closed when run returns
}

// newReportStream starts streaming from addr until close is called.
func newReportStream(addr string, logger logging.Logger) *reportStream {
	return startReportStream(&reportStream{
		addr:         addr,
		logger:       logger,
		staleAfter:   reportStaleAfter,
		retryInitial: reportRetryInitial,
		retryCeiling: reportRetryCeiling,
	})
}

func startReportStream(s *reportStream) *reportStream {
	ctx, cancel := context.WithCancel(context.Background())
	s.cancel = cancel
	s.done = make(chan struct{})
	go func() {
		defer close(s.done)
		s.run(ctx)
	}()
	return s
}

// close stops the background goroutine and waits for it to exit, so nothing
// logs or holds a socket after the arm is closed. Safe to call on a nil
// stream and more than once.
func (s *reportStream) close() {
	if s == nil {
		return
	}
	s.cancel()
	<-s.done
}

// run reconnects with exponential backoff until ctx is cancelled. A controller
// reboot drops the connection, so reconnecting is the normal path, not an
// exceptional one.
func (s *reportStream) run(ctx context.Context) {
	backoff := s.retryInitial
	first := true
	for {
		gotPacket, err := s.stream(ctx)
		if ctx.Err() != nil {
			return
		}
		// Warn when a working connection drops or the very first attempt fails;
		// stay at debug while it keeps failing so an unreachable port doesn't
		// flood the logs.
		if gotPacket || first {
			s.logger.Warnf("controller report stream on %s failed, reconnecting: %v", s.addr, err)
			backoff = s.retryInitial
		} else {
			s.logger.Debugf("controller report stream on %s still failing: %v", s.addr, err)
		}
		first = false

		select {
		case <-ctx.Done():
			return
		case <-time.After(backoff):
		}
		backoff = min(2*backoff, s.retryCeiling)
	}
}

// stream reads packets from one connection until it fails. gotPacket reports
// whether at least one packet was stored, which run uses to tell a dropped
// connection from one that never came up.
func (s *reportStream) stream(ctx context.Context) (gotPacket bool, err error) {
	dialCtx, cancel := context.WithTimeout(ctx, reportDialTimeout)
	defer cancel()
	var d net.Dialer
	conn, err := d.DialContext(dialCtx, "tcp", s.addr)
	if err != nil {
		return false, err
	}
	defer func() { _ = conn.Close() }() //nolint:errcheck // read-only socket; nothing to recover on close
	// Unblock the read below as soon as the stream is closed.
	stop := context.AfterFunc(ctx, func() { _ = conn.Close() }) //nolint:errcheck
	defer stop()

	for {
		// A controller that stops sending without closing the socket would
		// otherwise leave the stream blocked on a dead connection forever.
		if err = conn.SetReadDeadline(time.Now().Add(s.staleAfter)); err != nil {
			return gotPacket, err
		}
		var pkt []byte
		if pkt, err = readReportPacket(conn); err != nil {
			return gotPacket, err
		}
		// A packet that fails to decode means the stream has lost its framing,
		// so reconnect rather than keep reading at the wrong offset.
		var r controllerReport
		if r, err = decodeReport(pkt); err != nil {
			return gotPacket, err
		}
		s.store(r)
		if !gotPacket {
			gotPacket = true
			s.logger.Debugf("controller report stream on %s connected", s.addr)
		}
	}
}

func (s *reportStream) store(r controllerReport) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.latest = r
	s.received = time.Now()
}

// waitForFirstReport blocks until a fresh packet is available, so callers right
// after startup don't see errNoReport while the connection is still opening.
func (s *reportStream) waitForFirstReport(ctx context.Context) error {
	t := time.NewTicker(reportPollInterval)
	defer t.Stop()
	for {
		_, err := s.latestReport()
		if err == nil {
			return nil
		}
		select {
		case <-ctx.Done():
			return fmt.Errorf("waiting for a controller report: %w (%w)", err, ctx.Err())
		case <-t.C:
		}
	}
}

// latestReport returns a copy of the most recent packet, or an error if there
// is none fresh enough to trust. Every reader goes through it so locking and
// staleness are handled in one place.
func (s *reportStream) latestReport() (controllerReport, error) {
	if s == nil {
		return controllerReport{}, errNoReport
	}
	s.mu.RLock()
	defer s.mu.RUnlock()
	if s.received.IsZero() {
		return controllerReport{}, errNoReport
	}
	if age := time.Since(s.received); age > s.staleAfter {
		return controllerReport{}, fmt.Errorf("latest controller report is %v old", age.Round(time.Millisecond))
	}
	return s.latest, nil
}

// tcpLoad returns the payload the controller is currently compensating for.
func (s *reportStream) tcpLoad() (reportedTCPLoad, error) {
	r, err := s.latestReport()
	if err != nil {
		return reportedTCPLoad{}, err
	}
	return r.TCPLoad, nil
}

// readReportPacket reads one length-prefixed packet, prefix included. The
// length is big-endian, unlike the little-endian values inside the packet.
func readReportPacket(r io.Reader) ([]byte, error) {
	var prefix [reportLengthSize]byte
	if _, err := io.ReadFull(r, prefix[:]); err != nil {
		return nil, err
	}
	n := binary.BigEndian.Uint32(prefix[:])
	if n < reportMinLength || n > reportMaxLength {
		return nil, fmt.Errorf("report length %d outside [%d, %d]", n, reportMinLength, reportMaxLength)
	}
	pkt := make([]byte, n)
	copy(pkt, prefix[:])
	if _, err := io.ReadFull(r, pkt[reportLengthSize:]); err != nil {
		return nil, err
	}
	return pkt, nil
}

// decodeReport parses a packet from readReportPacket. It rejects the same
// out-of-range values the Python SDK does, since those mean a misframed or
// corrupt packet rather than a real controller state.
func decodeReport(pkt []byte) (controllerReport, error) {
	if len(pkt) < reportMinLength {
		return controllerReport{}, fmt.Errorf("report is %d bytes, need at least %d", len(pkt), reportMinLength)
	}
	r := controllerReport{
		State: pkt[reportOffsetStateMode] & 0x0F,
		Mode:  pkt[reportOffsetStateMode] >> 4,
		TCPLoad: reportedTCPLoad{
			MassKg: reportFloat32(pkt, reportOffsetTCPLoad),
			CenterOfGravityMM: r3.Vector{
				X: reportFloat32(pkt, reportOffsetTCPLoad+4),
				Y: reportFloat32(pkt, reportOffsetTCPLoad+8),
				Z: reportFloat32(pkt, reportOffsetTCPLoad+12),
			},
		},
	}
	if r.State > 9 || r.Mode > 11 || pkt[reportOffsetCollisSens] > 5 || pkt[reportOffsetTeachSens] > 5 {
		return controllerReport{}, fmt.Errorf("report has out-of-range fields (state=%d mode=%d collision_sens=%d teach_sens=%d)",
			r.State, r.Mode, pkt[reportOffsetCollisSens], pkt[reportOffsetTeachSens])
	}
	return r, nil
}

func reportFloat32(pkt []byte, off int) float64 {
	return float64(math.Float32frombits(binary.LittleEndian.Uint32(pkt[off : off+4])))
}

func reportAddr(host string) string {
	return net.JoinHostPort(host, strconv.Itoa(defaultReportPort))
}
