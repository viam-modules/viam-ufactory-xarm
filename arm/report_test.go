package arm

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"math"
	"net"
	"os"
	"slices"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

// buildReport returns a rich report packet of the given length with the fields
// decodeReport reads filled in.
func buildReport(length int, state, mode byte, load reportedTCPLoad) []byte {
	pkt := make([]byte, length)
	binary.BigEndian.PutUint32(pkt, uint32(length))
	pkt[reportOffsetStateMode] = mode<<4 | state
	for i, v := range []float64{load.MassKg, load.CenterOfGravityMM.X, load.CenterOfGravityMM.Y, load.CenterOfGravityMM.Z} {
		off := reportOffsetTCPLoad + 4*i
		binary.LittleEndian.PutUint32(pkt[off:off+4], math.Float32bits(float32(v)))
	}
	pkt[reportOffsetCollisSens] = 3
	pkt[reportOffsetTeachSens] = 1
	return pkt
}

var g1Load = reportedTCPLoad{MassKg: 0.82, CenterOfGravityMM: r3.Vector{X: 1.5, Y: -2, Z: 48}}

func TestDecodeReport(t *testing.T) {
	r, err := decodeReport(buildReport(587, 2, 1, g1Load))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, r.State, test.ShouldEqual, 2)
	test.That(t, r.Mode, test.ShouldEqual, 1)
	test.That(t, r.TCPLoad.MassKg, test.ShouldAlmostEqual, 0.82, 1e-6)
	test.That(t, r.TCPLoad.CenterOfGravityMM, test.ShouldResemble, r3.Vector{X: 1.5, Y: -2, Z: 48})

	t.Run("status fields", func(t *testing.T) {
		pkt := buildReport(587, 2, 1, g1Load)
		binary.BigEndian.PutUint16(pkt[reportOffsetCmdNum:], 0x0102)
		pkt[reportOffsetBrake] = 0x3F
		pkt[reportOffsetEnable] = 0x1F
		pkt[reportOffsetErrorCode] = 0x16
		pkt[reportOffsetWarnCode] = 0x0B
		r, err := decodeReport(pkt)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, r.CmdNum, test.ShouldEqual, 0x0102)
		test.That(t, r.BrakeMask, test.ShouldEqual, 0x3F)
		test.That(t, r.EnableMask, test.ShouldEqual, 0x1F)
		test.That(t, r.ErrorCode, test.ShouldEqual, 0x16)
		test.That(t, r.WarnCode, test.ShouldEqual, 0x0B)
	})

	t.Run("too short", func(t *testing.T) {
		_, err := decodeReport(buildReport(reportMinLength, 0, 0, g1Load)[:reportMinLength-1])
		test.That(t, err, test.ShouldNotBeNil)
	})

	for name, mutate := range map[string]func([]byte){
		"bad state":       func(p []byte) { p[reportOffsetStateMode] = 0x0A },
		"bad mode":        func(p []byte) { p[reportOffsetStateMode] = 0xC0 },
		"bad collis sens": func(p []byte) { p[reportOffsetCollisSens] = 6 },
		"bad teach sens":  func(p []byte) { p[reportOffsetTeachSens] = 6 },
	} {
		t.Run(name, func(t *testing.T) {
			pkt := buildReport(587, 0, 0, g1Load)
			mutate(pkt)
			_, err := decodeReport(pkt)
			test.That(t, err, test.ShouldNotBeNil)
		})
	}
}

func TestReadyForMotion(t *testing.T) {
	ready := controllerReport{State: reportStateSleeping, BrakeMask: 0x3F, EnableMask: 0x3F}
	for name, tc := range map[string]struct {
		r    func(controllerReport) controllerReport
		axes int
		want bool
	}{
		"idle with every joint live": {func(r controllerReport) controllerReport { return r }, 6, true},
		"moving":                     {func(r controllerReport) controllerReport { r.State = reportStateMoving; return r }, 6, true},
		"state 0":                    {func(r controllerReport) controllerReport { r.State = 0; return r }, 6, true},
		"paused":                     {func(r controllerReport) controllerReport { r.State = 3; return r }, 6, false},
		"stopped":                    {func(r controllerReport) controllerReport { r.State = 4; return r }, 6, false},
		"not ready":                  {func(r controllerReport) controllerReport { r.State = 5; return r }, 6, false},
		"one brake engaged":          {func(r controllerReport) controllerReport { r.BrakeMask = 0x3B; return r }, 6, false},
		"one servo disabled":         {func(r controllerReport) controllerReport { r.EnableMask = 0x1F; return r }, 6, false},
		"seventh joint not live":     {func(r controllerReport) controllerReport { return r }, 7, false},
		"unused bits ignored":        {func(r controllerReport) controllerReport { r.BrakeMask = 0xBF; return r }, 6, true},
	} {
		t.Run(name, func(t *testing.T) {
			test.That(t, tc.r(ready).readyForMotion(tc.axes), test.ShouldEqual, tc.want)
		})
	}
}

func TestReadReportPacket(t *testing.T) {
	t.Run("reads exactly one packet", func(t *testing.T) {
		first := buildReport(245, 0, 1, g1Load)
		second := buildReport(245, 4, 0, reportedTCPLoad{})
		pkt, err := readReportPacket(bytes.NewReader(append(append([]byte{}, first...), second...)))
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pkt, test.ShouldResemble, first)
	})

	t.Run("length out of range", func(t *testing.T) {
		for _, n := range []uint32{reportMinLength - 1, reportMaxLength + 1} {
			var prefix [4]byte
			binary.BigEndian.PutUint32(prefix[:], n)
			_, err := readReportPacket(bytes.NewReader(prefix[:]))
			test.That(t, err, test.ShouldNotBeNil)
		}
	})

	t.Run("truncated body", func(t *testing.T) {
		_, err := readReportPacket(bytes.NewReader(buildReport(245, 0, 0, g1Load)[:200]))
		test.That(t, err, test.ShouldNotBeNil)
	})
}

// fakeReportServer accepts report connections and, for each one, sends the
// packets from the next entry of conns, then closes it. Once conns runs out,
// the connection is held open and silent until the server closes.
type fakeReportServer struct {
	ln   net.Listener
	done chan struct{}
}

func newFakeReportServer(t *testing.T, conns ...[][]byte) *fakeReportServer {
	t.Helper()
	var lc net.ListenConfig
	ln, err := lc.Listen(context.Background(), "tcp", "127.0.0.1:0")
	test.That(t, err, test.ShouldBeNil)
	s := &fakeReportServer{ln: ln, done: make(chan struct{})}
	go func() {
		for i := 0; ; i++ {
			c, err := ln.Accept()
			if err != nil {
				return
			}
			if i >= len(conns) {
				go func() {
					<-s.done
					_ = c.Close()
				}()
				continue
			}
			for _, pkt := range conns[i] {
				// Split each packet across two writes so the reader has to reassemble it.
				_, _ = c.Write(pkt[:100])
				_, _ = c.Write(pkt[100:])
			}
			_ = c.Close()
		}
	}()
	t.Cleanup(func() {
		close(s.done)
		_ = ln.Close()
	})
	return s
}

func newTestReportStream(t *testing.T, addr string, staleAfter time.Duration) *reportStream {
	t.Helper()
	s := &reportStream{
		addr:         addr,
		logger:       logging.NewTestLogger(t),
		staleAfter:   staleAfter,
		retryInitial: 10 * time.Millisecond,
		retryCeiling: 50 * time.Millisecond,
	}
	test.That(t, s.start(context.Background()), test.ShouldBeNil)
	t.Cleanup(s.close)
	return s
}

func waitForTCPLoadMass(t *testing.T, s *reportStream, want float64) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for {
		load, err := s.tcpLoad()
		if err == nil && math.Abs(load.MassKg-want) < 1e-6 {
			return
		}
		if time.Now().After(deadline) {
			t.Fatalf("tcpLoad never reported mass %v; last: %+v, %v", want, load, err)
		}
		time.Sleep(5 * time.Millisecond)
	}
}

func TestReportStream(t *testing.T) {
	t.Run("streams the latest packet", func(t *testing.T) {
		srv := newFakeReportServer(t, [][]byte{
			buildReport(587, 0, 1, reportedTCPLoad{MassKg: 0.5}),
			buildReport(587, 0, 1, g1Load),
		})
		s := newTestReportStream(t, srv.ln.Addr().String(), time.Second)

		test.That(t, s.waitForFirstReport(context.Background(), 2*time.Second), test.ShouldBeNil)
		waitForTCPLoadMass(t, s, 0.82)

		r, err := s.latestReport()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, r.Mode, test.ShouldEqual, 1)
		test.That(t, r.TCPLoad.CenterOfGravityMM, test.ShouldResemble, r3.Vector{X: 1.5, Y: -2, Z: 48})
	})

	t.Run("reconnects after the controller drops the connection", func(t *testing.T) {
		srv := newFakeReportServer(t,
			[][]byte{buildReport(587, 0, 0, g1Load)},
			[][]byte{buildReport(587, 0, 0, reportedTCPLoad{MassKg: 0.61, CenterOfGravityMM: r3.Vector{Z: 53}})},
		)
		s := newTestReportStream(t, srv.ln.Addr().String(), time.Second)
		waitForTCPLoadMass(t, s, 0.61)
	})

	t.Run("reconnects after a misframed packet", func(t *testing.T) {
		bad := buildReport(587, 0, 0, g1Load)
		bad[reportOffsetStateMode] = 0xFF
		srv := newFakeReportServer(t,
			[][]byte{bad, buildReport(587, 0, 0, reportedTCPLoad{MassKg: 0.5})},
			[][]byte{buildReport(587, 0, 0, g1Load)},
		)
		s := newTestReportStream(t, srv.ln.Addr().String(), time.Second)
		waitForTCPLoadMass(t, s, 0.82)
	})

	t.Run("goes stale when the controller stops sending", func(t *testing.T) {
		srv := newFakeReportServer(t, [][]byte{buildReport(587, 0, 0, g1Load)})
		s := newTestReportStream(t, srv.ln.Addr().String(), 100*time.Millisecond)
		waitForTCPLoadMass(t, s, 0.82)

		time.Sleep(200 * time.Millisecond)
		_, err := s.tcpLoad()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "old")
	})

	t.Run("no report before the first packet", func(t *testing.T) {
		srv := newFakeReportServer(t) // accepts but never sends
		s := newTestReportStream(t, srv.ln.Addr().String(), time.Second)
		_, err := s.tcpLoad()
		test.That(t, errors.Is(err, errNoReport), test.ShouldBeTrue)

		test.That(t, s.waitForFirstReport(context.Background(), 100*time.Millisecond), test.ShouldNotBeNil)
	})

	t.Run("fails fast when the port is unreachable", func(t *testing.T) {
		var lc net.ListenConfig
		ln, err := lc.Listen(context.Background(), "tcp", "127.0.0.1:0")
		test.That(t, err, test.ShouldBeNil)
		addr := ln.Addr().String()
		test.That(t, ln.Close(), test.ShouldBeNil)

		start := time.Now()
		s, err := newReportStream(context.Background(), addr, logging.NewTestLogger(t))
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, s, test.ShouldBeNil)
		test.That(t, time.Since(start), test.ShouldBeLessThan, reportDialTimeout)
	})

	t.Run("recovers after a connection that sends nothing", func(t *testing.T) {
		// The second connection closes without a packet, the path run logs at
		// debug; the third one delivers again.
		srv := newFakeReportServer(t,
			[][]byte{buildReport(587, 0, 0, g1Load)},
			[][]byte{},
			[][]byte{buildReport(587, 0, 0, reportedTCPLoad{MassKg: 0.3})},
		)
		s := newTestReportStream(t, srv.ln.Addr().String(), time.Second)
		waitForTCPLoadMass(t, s, 0.3)
	})

	t.Run("close returns while blocked on a silent connection", func(t *testing.T) {
		srv := newFakeReportServer(t)
		s := newTestReportStream(t, srv.ln.Addr().String(), time.Second)
		time.Sleep(50 * time.Millisecond) // let it connect and block on the read

		closed := make(chan struct{})
		go func() {
			s.close()
			close(closed)
		}()
		select {
		case <-closed:
		case <-time.After(time.Second):
			t.Fatal("close did not return")
		}
		s.close() // a second close is a no-op
	})

	t.Run("nil stream", func(t *testing.T) {
		var s *reportStream
		_, err := s.tcpLoad()
		test.That(t, errors.Is(err, errNoReport), test.ShouldBeTrue)
		_, err = s.waitForReport(context.Background(), time.Time{}, time.Second, nil)
		test.That(t, errors.Is(err, errNoReport), test.ShouldBeTrue)
		s.close()
	})
}

// newStoredReportStream returns a stream with no connection whose packets the
// test stores directly.
func newStoredReportStream(t *testing.T) *reportStream {
	t.Helper()
	return &reportStream{logger: logging.NewTestLogger(t), staleAfter: time.Second}
}

func TestWaitForReport(t *testing.T) {
	ctx := context.Background()

	t.Run("accepts a packet received after since", func(t *testing.T) {
		s := newStoredReportStream(t)
		since := time.Now()
		s.store(controllerReport{Mode: 1})
		r, err := s.waitForReport(ctx, since, time.Second, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, r.Mode, test.ShouldEqual, 1)
	})

	t.Run("waits for a packet received after since", func(t *testing.T) {
		s := newStoredReportStream(t)
		s.store(controllerReport{Mode: 1})
		since := time.Now()
		go func() {
			time.Sleep(100 * time.Millisecond)
			s.store(controllerReport{Mode: 2})
		}()
		r, err := s.waitForReport(ctx, since, time.Second, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, r.Mode, test.ShouldEqual, 2)
	})

	t.Run("waits for a packet that matches", func(t *testing.T) {
		s := newStoredReportStream(t)
		s.store(controllerReport{Mode: 1})
		go func() {
			time.Sleep(100 * time.Millisecond)
			s.store(controllerReport{Mode: 2})
		}()
		r, err := s.waitForReport(ctx, time.Time{}, time.Second, func(r controllerReport) bool { return r.Mode == 2 })
		test.That(t, err, test.ShouldBeNil)
		test.That(t, r.Mode, test.ShouldEqual, 2)
	})

	t.Run("returns the last packet on timeout", func(t *testing.T) {
		s := newStoredReportStream(t)
		s.store(controllerReport{Mode: 1, State: 4})
		start := time.Now()
		r, err := s.waitForReport(ctx, time.Time{}, 150*time.Millisecond, func(r controllerReport) bool { return r.Mode == 2 })
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, time.Since(start), test.ShouldBeGreaterThanOrEqualTo, 150*time.Millisecond)
		test.That(t, r.Mode, test.ShouldEqual, 1)
		test.That(t, r.State, test.ShouldEqual, 4)
	})

	t.Run("times out with no packet since", func(t *testing.T) {
		s := newStoredReportStream(t)
		s.store(controllerReport{Mode: 1})
		_, err := s.waitForReport(ctx, time.Now(), 100*time.Millisecond, nil)
		test.That(t, err, test.ShouldNotBeNil)
	})
}

// TestReportRateOnHardware measures how often a real controller sends rich
// report packets. It runs only when XARM_HOST names a reachable controller:
//
//	XARM_HOST=192.168.1.10 go test ./arm -run TestReportRateOnHardware -v
func TestReportRateOnHardware(t *testing.T) {
	host := os.Getenv("XARM_HOST")
	if host == "" {
		t.Skip("set XARM_HOST to a controller's address to measure its report rate")
	}
	const packets = 100

	var d net.Dialer
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	conn, err := d.DialContext(ctx, "tcp", reportAddr(host))
	test.That(t, err, test.ShouldBeNil)
	defer func() { _ = conn.Close() }()
	test.That(t, conn.SetReadDeadline(time.Now().Add(30*time.Second)), test.ShouldBeNil)

	// The first packet may have been queued before we connected, so start
	// timing from it rather than from the dial.
	pkt, err := readReportPacket(conn)
	test.That(t, err, test.ShouldBeNil)
	prev := time.Now()
	intervals := make([]time.Duration, 0, packets)
	for range packets {
		pkt, err = readReportPacket(conn)
		test.That(t, err, test.ShouldBeNil)
		now := time.Now()
		intervals = append(intervals, now.Sub(prev))
		prev = now
	}
	r, err := decodeReport(pkt)
	test.That(t, err, test.ShouldBeNil)

	slices.Sort(intervals)
	var total time.Duration
	for _, iv := range intervals {
		total += iv
	}
	mean := total / time.Duration(len(intervals))
	t.Logf("%d packets of %d bytes: mean interval %v (%.1f Hz), min %v, median %v, p95 %v, max %v",
		len(intervals), len(pkt), mean.Round(time.Microsecond), float64(time.Second)/float64(mean),
		intervals[0].Round(time.Microsecond), intervals[len(intervals)/2].Round(time.Microsecond),
		intervals[len(intervals)*95/100].Round(time.Microsecond), intervals[len(intervals)-1].Round(time.Microsecond))
	t.Logf("last packet: state=%d mode=%d cmdnum=%d error=%d warn=%d brake=%08b enable=%08b",
		r.State, r.Mode, r.CmdNum, r.ErrorCode, r.WarnCode, r.BrakeMask, r.EnableMask)
}
