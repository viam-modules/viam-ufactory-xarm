package arm

import (
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"strconv"
	"time"

	"github.com/golang/geo/r3"
)

// The controller pushes a status packet on each of its report ports to every
// connected client; nothing is sent to it. Port 30002 ("rich") carries the
// most fields, including configuration such as the TCP payload that the
// command socket on port 502 has no register to read.
const (
	defaultReportPort = 30002
	reportReadTimeout = 3 * time.Second
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

// reportedTCPLoad is the payload the controller is currently compensating for,
// whichever source set it: UFACTORY Studio, the saved controller config, or a
// SetLoadParam call.
type reportedTCPLoad struct {
	MassKg float64
	// CenterOfGravityMM is relative to the flange in the tool frame. Firmware
	// older than 0.2.1 reported metres; no arm this module supports runs it.
	CenterOfGravityMM r3.Vector
}

// controllerReport is the part of a rich report packet the module reads.
type controllerReport struct {
	State   byte
	Mode    byte
	TCPLoad reportedTCPLoad
}

// readReport connects to a report port at addr, reads one packet, and closes
// the connection.
func readReport(ctx context.Context, addr string) (controllerReport, error) {
	ctx, cancel := context.WithTimeout(ctx, reportReadTimeout)
	defer cancel()

	var d net.Dialer
	conn, err := d.DialContext(ctx, "tcp", addr)
	if err != nil {
		return controllerReport{}, fmt.Errorf("failed to connect to report port %s: %w", addr, err)
	}
	defer func() { _ = conn.Close() }() //nolint:errcheck // read-only socket; nothing to recover on close
	if deadline, ok := ctx.Deadline(); ok {
		if err := conn.SetReadDeadline(deadline); err != nil {
			return controllerReport{}, err
		}
	}

	pkt, err := readReportPacket(conn)
	if err != nil {
		return controllerReport{}, fmt.Errorf("failed to read report from %s: %w", addr, err)
	}
	return decodeReport(pkt)
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

// readReport reads one rich report from this arm's controller.
func (x *xArm) readReport(ctx context.Context) (controllerReport, error) {
	return readReport(ctx, net.JoinHostPort(x.conf.Host, strconv.Itoa(defaultReportPort)))
}
