package arm

import (
	"bytes"
	"context"
	"encoding/binary"
	"math"
	"net"
	"testing"

	"github.com/golang/geo/r3"
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

func TestReadReport(t *testing.T) {
	var lc net.ListenConfig
	ln, err := lc.Listen(context.Background(), "tcp", "127.0.0.1:0")
	test.That(t, err, test.ShouldBeNil)
	defer func() { test.That(t, ln.Close(), test.ShouldBeNil) }()

	// Send the packet in two writes so the reader has to reassemble it.
	pkt := buildReport(587, 0, 1, g1Load)
	go func() {
		c, err := ln.Accept()
		if err != nil {
			return
		}
		defer func() { _ = c.Close() }()
		_, _ = c.Write(pkt[:100])
		_, _ = c.Write(pkt[100:])
	}()

	r, err := readReport(context.Background(), ln.Addr().String())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, r.Mode, test.ShouldEqual, 1)
	test.That(t, r.TCPLoad.MassKg, test.ShouldAlmostEqual, 0.82, 1e-6)
	test.That(t, r.TCPLoad.CenterOfGravityMM.Z, test.ShouldEqual, 48)
}
