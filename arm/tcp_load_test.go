package arm

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"
)

func TestValidateTCPLoad(t *testing.T) {
	for _, tc := range []struct {
		name    string
		load    tcpLoad
		wantErr bool
	}{
		{"zero is valid", tcpLoad{}, false},
		{"typical gripper", tcpLoad{massKg: 0.82, cogMM: r3.Vector{Z: 48}}, false},
		{"negative mass", tcpLoad{massKg: -0.1}, true},
		{"NaN mass", tcpLoad{massKg: math.NaN()}, true},
		{"Inf mass", tcpLoad{massKg: math.Inf(1)}, true},
		{"NaN in cog", tcpLoad{massKg: 1, cogMM: r3.Vector{X: math.NaN()}}, true},
		{"Inf in cog", tcpLoad{massKg: 1, cogMM: r3.Vector{Z: math.Inf(-1)}}, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := tc.load.validate()
			if tc.wantErr {
				test.That(t, err, test.ShouldNotBeNil)
			} else {
				test.That(t, err, test.ShouldBeNil)
			}
		})
	}
}

func TestFirmwareUsesMillimeters(t *testing.T) {
	for _, tc := range []struct {
		version string
		want    bool
	}{
		{"2.5.0", true},
		{"1.11.100", true},
		{"0.2.1", true},  // boundary: >= 0.2.1 is mm
		{"0.2.0", false}, // just below
		{"0.1.9", false},
		{"0.0.0", false},
		{"", true},        // unknown defaults to mm
		{"garbage", true}, // unparseable defaults to mm
		{"2.5", true},     // malformed defaults to mm
	} {
		t.Run(tc.version, func(t *testing.T) {
			test.That(t, firmwareUsesMillimeters(tc.version), test.ShouldEqual, tc.want)
		})
	}
}
