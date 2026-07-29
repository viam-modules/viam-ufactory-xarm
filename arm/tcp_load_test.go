package arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	rutils "go.viam.com/rdk/utils"
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
		{"mass beyond float32 range", tcpLoad{massKg: 3.5e38}, true},
		{"cog beyond float32 range", tcpLoad{massKg: 1, cogMM: r3.Vector{Y: -3.5e38}}, true},
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
		{"0.10.0", true},  // 10 > 2 numerically, but "0.10.0" < "0.2.1" as a string
		{"0.2.10", true},  // 10 > 1 numerically, but "0.2.10" < "0.2.1" as a string
		{"0.3.0", true},   // just above the boundary on the minor
		{"0.2.2", true},   // just above the boundary on the patch
		{"", true},        // unknown defaults to mm
		{"garbage", true}, // unparseable defaults to mm
		{"2.5", true},     // malformed defaults to mm
		{"-1.0.0", true},  // negative component defaults to mm
	} {
		t.Run(fmt.Sprintf("%q", tc.version), func(t *testing.T) {
			test.That(t, firmwareUsesMillimeters(tc.version), test.ShouldEqual, tc.want)
		})
	}
}

func TestSetTCPLoadOpcode(t *testing.T) {
	// Pins the opcode transcribed from the external SDK. regMap already has a
	// legitimate duplicate (SetBound/EnableBound both 0x34), so a typo here
	// (e.g. to 0x25) would silently alias Sensitivity and misdirect the wire
	// payload instead of failing loudly.
	test.That(t, regMap["SetTCPLoad"], test.ShouldEqual, byte(0x24))
}

func TestEncodeTCPLoad(t *testing.T) {
	l := tcpLoad{massKg: 0.82, cogMM: r3.Vector{X: 1, Y: 2, Z: 48}}

	// Modern firmware: mm passed through unchanged.
	got := encodeTCPLoad(l, true)
	test.That(t, len(got), test.ShouldEqual, 16)
	for i, want := range []float64{0.82, 1, 2, 48} {
		f := rutils.Float32FromBytesLE(got[i*4 : i*4+4])
		test.That(t, float64(f), test.ShouldAlmostEqual, want, 1e-5)
	}

	// Legacy firmware: center of gravity converted to meters, mass untouched.
	got = encodeTCPLoad(l, false)
	test.That(t, len(got), test.ShouldEqual, 16)
	for i, want := range []float64{0.82, 0.001, 0.002, 0.048} {
		f := rutils.Float32FromBytesLE(got[i*4 : i*4+4])
		test.That(t, float64(f), test.ShouldAlmostEqual, want, 1e-7)
	}
}

func TestRatedPayloadKg(t *testing.T) {
	for _, tc := range []struct {
		model hardwareModel
		want  float64
		ok    bool
	}{
		{hardwareModelLite6, 0.5, true},
		{hardwareModelXArm5, 3.0, true},
		{hardwareModelXArm7, 3.5, true},
		{hardwareModelXArm7T, 3.5, true},
		{hardwareModelXArm6, 5.0, true},
		{hardwareModelXArm850, 5.0, true},
		{hardwareModelUnknown, 0, false},
	} {
		t.Run(string(tc.model), func(t *testing.T) {
			got, ok := ratedPayloadKg(tc.model)
			test.That(t, ok, test.ShouldEqual, tc.ok)
			if tc.ok {
				test.That(t, got, test.ShouldAlmostEqual, tc.want, 1e-9)
			}
		})
	}
}

func TestGripperDefaultTCPLoad(t *testing.T) {
	for _, tc := range []struct {
		name  string
		model resource.Model
		want  tcpLoad
		ok    bool
	}{
		{"standard gripper", GripperModel, tcpLoad{massKg: 0.82, cogMM: r3.Vector{Z: 48}}, true},
		{"vacuum gripper", VacuumGripperModel, tcpLoad{massKg: 0.61, cogMM: r3.Vector{Z: 53}}, true},
		// Lite variants have no published preset and must never push a default:
		// the xArm presets would exceed the Lite6's 0.5 kg total rating.
		{"vacuum gripper lite", VacuumGripperModelLite, tcpLoad{}, false},
		{"gripper lite", GripperModelLite, tcpLoad{}, false},
		{"unrelated model", FTSensorModel, tcpLoad{}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, ok := gripperDefaultTCPLoad(tc.model)
			test.That(t, ok, test.ShouldEqual, tc.ok)
			if tc.ok {
				test.That(t, got, test.ShouldResemble, tc.want)
			}
		})
	}
}

// Checks that the published gripper presets pass tcpLoad.validate() (finite,
// non-negative mass). It does NOT check them against any arm's rated payload —
// both presets (0.82, 0.61) in fact exceed the Lite6's 0.5 kg rating, which is
// exactly why gripperDefaultTCPLoad refuses to return a default for the Lite
// variants, and why Task 6 adds a separate rating-refusal check.
func TestGripperDefaultsValidate(t *testing.T) {
	for _, m := range []resource.Model{GripperModel, VacuumGripperModel} {
		l, ok := gripperDefaultTCPLoad(m)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, l.validate(), test.ShouldBeNil)
	}
}

func TestApplyTCPLoadPrecedence(t *testing.T) {
	// A default applies only when nothing has been written yet.
	t.Run("default applies when unset", func(t *testing.T) {
		test.That(t, shouldApplyTCPLoad(tcpLoadSourceUnset, tcpLoadSourceGripperDefault), test.ShouldBeTrue)
	})
	t.Run("default suppressed by config", func(t *testing.T) {
		test.That(t, shouldApplyTCPLoad(tcpLoadSourceConfig, tcpLoadSourceGripperDefault), test.ShouldBeFalse)
	})
	// The case that corrupts a live grasp if it regresses: a gripper rebuild
	// must not clobber a runtime set_tcp_load.
	t.Run("default suppressed by runtime set", func(t *testing.T) {
		test.That(t, shouldApplyTCPLoad(tcpLoadSourceDoCommand, tcpLoadSourceGripperDefault), test.ShouldBeFalse)
	})
	t.Run("default suppressed by earlier default", func(t *testing.T) {
		test.That(t, shouldApplyTCPLoad(tcpLoadSourceGripperDefault, tcpLoadSourceGripperDefault), test.ShouldBeFalse)
	})
	// Explicit writes always win, from any prior state.
	for _, prior := range []tcpLoadSource{
		tcpLoadSourceUnset, tcpLoadSourceConfig, tcpLoadSourceDoCommand, tcpLoadSourceGripperDefault,
	} {
		test.That(t, shouldApplyTCPLoad(prior, tcpLoadSourceDoCommand), test.ShouldBeTrue)
		test.That(t, shouldApplyTCPLoad(prior, tcpLoadSourceConfig), test.ShouldBeTrue)
	}
}

func TestTCPLoadSourceString(t *testing.T) {
	test.That(t, tcpLoadSourceUnset.String(), test.ShouldEqual, "unset")
	test.That(t, tcpLoadSourceConfig.String(), test.ShouldEqual, "config")
	test.That(t, tcpLoadSourceDoCommand.String(), test.ShouldEqual, "do_command")
	test.That(t, tcpLoadSourceGripperDefault.String(), test.ShouldEqual, "gripper_default")
}

func TestExceedsRating(t *testing.T) {
	// Over rating on a known model.
	over, rated := exceedsRating(tcpLoad{massKg: 0.61}, hardwareModelLite6)
	test.That(t, over, test.ShouldBeTrue)
	test.That(t, rated, test.ShouldAlmostEqual, 0.5, 1e-9)

	// At the rating exactly is not over.
	over, _ = exceedsRating(tcpLoad{massKg: 0.5}, hardwareModelLite6)
	test.That(t, over, test.ShouldBeFalse)

	// Unknown model cannot be checked.
	over, _ = exceedsRating(tcpLoad{massKg: 99}, hardwareModelUnknown)
	test.That(t, over, test.ShouldBeFalse)
}

func TestDecideTCPLoad(t *testing.T) {
	// The asymmetry the spec requires: over-rating warns for explicit writes but
	// refuses outright for a pushed default.
	t.Run("over-rating warns for config", func(t *testing.T) {
		d := decideTCPLoad(tcpLoad{massKg: 9}, tcpLoadSourceConfig, tcpLoadSourceUnset, hardwareModelLite6)
		test.That(t, d.action, test.ShouldEqual, tcpLoadActionApply)
		test.That(t, d.warnOverRating, test.ShouldBeTrue)
	})
	t.Run("over-rating warns for do_command", func(t *testing.T) {
		d := decideTCPLoad(tcpLoad{massKg: 9}, tcpLoadSourceDoCommand, tcpLoadSourceUnset, hardwareModelLite6)
		test.That(t, d.action, test.ShouldEqual, tcpLoadActionApply)
		test.That(t, d.warnOverRating, test.ShouldBeTrue)
	})
	t.Run("over-rating refuses a pushed default", func(t *testing.T) {
		d := decideTCPLoad(tcpLoad{massKg: 0.61}, tcpLoadSourceGripperDefault, tcpLoadSourceUnset, hardwareModelLite6)
		test.That(t, d.action, test.ShouldEqual, tcpLoadActionRefuse)
	})
	t.Run("in-rating default applies", func(t *testing.T) {
		d := decideTCPLoad(tcpLoad{massKg: 0.61}, tcpLoadSourceGripperDefault, tcpLoadSourceUnset, hardwareModelXArm6)
		test.That(t, d.action, test.ShouldEqual, tcpLoadActionApply)
		test.That(t, d.warnOverRating, test.ShouldBeFalse)
	})
	// Precedence is checked before rating, so a suppressed default never warns.
	t.Run("suppression beats rating", func(t *testing.T) {
		d := decideTCPLoad(tcpLoad{massKg: 9}, tcpLoadSourceGripperDefault, tcpLoadSourceDoCommand, hardwareModelLite6)
		test.That(t, d.action, test.ShouldEqual, tcpLoadActionSuppress)
	})
}

func TestApplyTCPLoadDoesNotCacheFailedWrite(t *testing.T) {
	x := &xArm{logger: logging.NewTestLogger(t)}
	boom := errors.New("controller rejected the write")

	err := x.applyTCPLoadWith(context.Background(), tcpLoad{massKg: 1.2}, tcpLoadSourceDoCommand, "test",
		func(context.Context, tcpLoad) error { return boom })

	test.That(t, err, test.ShouldNotBeNil)
	// The getter must never report a value the controller refused.
	test.That(t, x.tcpLoadSource, test.ShouldEqual, tcpLoadSourceUnset)
	test.That(t, x.tcpLoad.massKg, test.ShouldAlmostEqual, 0, 1e-9)
}

func TestApplyTCPLoadCachesSuccessfulWrite(t *testing.T) {
	x := &xArm{logger: logging.NewTestLogger(t)}

	err := x.applyTCPLoadWith(context.Background(), tcpLoad{massKg: 1.2}, tcpLoadSourceDoCommand, "test",
		func(context.Context, tcpLoad) error { return nil })

	test.That(t, err, test.ShouldBeNil)
	test.That(t, x.tcpLoadSource, test.ShouldEqual, tcpLoadSourceDoCommand)
	test.That(t, x.tcpLoad.massKg, test.ShouldAlmostEqual, 1.2, 1e-9)
}

func TestApplyTCPLoadSuppressedDefaultDoesNotWrite(t *testing.T) {
	x := &xArm{logger: logging.NewTestLogger(t)}
	x.tcpLoad = tcpLoad{massKg: 1.2}
	x.tcpLoadSource = tcpLoadSourceDoCommand
	x.tcpLoadRequester = "set_tcp_load"

	wrote := false
	err := x.applyTCPLoadWith(context.Background(), tcpLoad{massKg: 0.61}, tcpLoadSourceGripperDefault, "vacuum_gripper",
		func(context.Context, tcpLoad) error { wrote = true; return nil })

	// Suppression is not an error, but nothing may reach the controller and the
	// held payload must survive: this is the mid-grasp overwrite case.
	test.That(t, err, test.ShouldBeNil)
	test.That(t, wrote, test.ShouldBeFalse)
	test.That(t, x.tcpLoad.massKg, test.ShouldAlmostEqual, 1.2, 1e-9)
}
