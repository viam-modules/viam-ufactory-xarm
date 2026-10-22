package arm

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"math"
	"net"
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
		s.close()
	})
}
