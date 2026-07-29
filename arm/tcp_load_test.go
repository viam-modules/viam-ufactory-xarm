package arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	rutils "go.viam.com/rdk/utils"
)

// tcpLoadRecorderArm captures DoCommand calls. Embeds arm.Arm so only
// DoCommand needs implementing.
type tcpLoadRecorderArm struct {
	arm.Arm

	onDo func(map[string]any)
}

func (f *tcpLoadRecorderArm) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	if f.onDo != nil {
		f.onDo(cmd)
	}
	return map[string]any{}, nil
}

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
		// "" is the actual value x.detectedArm.model holds when detectArm fails
		// (xarm.go discards the returned detectedArm on error); hardwareModelUnknown
		// is never reached on that path. Both must land on ratedPayloadKg's default.
		{hardwareModel(""), 0, false},
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

// TestApplyConfigTCPLoad pins the seam NewXArm calls: given a *TCPLoadConfig,
// it must invoke apply with source tcpLoadSourceConfig and requester "config"
// — not any other source, since that would invert the precedence model Tasks
// 5-6 establish (e.g. a config-set payload on a Lite6 would be refused instead
// of applied-with-warning, and config would stop suppressing gripper pushes).
// A nil config must not call apply at all.
func TestApplyConfigTCPLoad(t *testing.T) {
	t.Run("nil config applies nothing", func(t *testing.T) {
		called := false
		err := applyConfigTCPLoad(context.Background(), nil,
			func(context.Context, tcpLoad, tcpLoadSource, string) error {
				called = true
				return nil
			})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, called, test.ShouldBeFalse)
	})

	t.Run("wires source and requester", func(t *testing.T) {
		var gotLoad tcpLoad
		var gotSrc tcpLoadSource
		var gotRequester string
		err := applyConfigTCPLoad(context.Background(), &TCPLoadConfig{MassKg: massKgPtr(0.82)},
			func(_ context.Context, l tcpLoad, src tcpLoadSource, requester string) error {
				gotLoad, gotSrc, gotRequester = l, src, requester
				return nil
			})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, gotLoad.massKg, test.ShouldAlmostEqual, 0.82, 1e-9)
		test.That(t, gotSrc, test.ShouldEqual, tcpLoadSourceConfig)
		test.That(t, gotRequester, test.ShouldEqual, "config")
	})

	t.Run("propagates conversion errors without calling apply", func(t *testing.T) {
		called := false
		err := applyConfigTCPLoad(context.Background(), &TCPLoadConfig{},
			func(context.Context, tcpLoad, tcpLoadSource, string) error {
				called = true
				return nil
			})
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, called, test.ShouldBeFalse)
	})

	t.Run("propagates apply errors", func(t *testing.T) {
		boom := errors.New("write failed")
		err := applyConfigTCPLoad(context.Background(), &TCPLoadConfig{MassKg: massKgPtr(0.82)},
			func(context.Context, tcpLoad, tcpLoadSource, string) error {
				return boom
			})
		test.That(t, errors.Is(err, boom), test.ShouldBeTrue)
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

func TestApplyTCPLoadRefusesOverRatingDefault(t *testing.T) {
	x := &xArm{logger: logging.NewTestLogger(t)}
	x.detectedArm = detectedArm{model: hardwareModelLite6}

	wrote := false
	err := x.applyTCPLoadWith(context.Background(), tcpLoad{massKg: 0.61},
		tcpLoadSourceGripperDefault, "vacuum_gripper",
		func(context.Context, tcpLoad) error { wrote = true; return nil })

	// Refusal must not fail the gripper's construction, must not reach the
	// controller, and must not be cached.
	test.That(t, err, test.ShouldBeNil)
	test.That(t, wrote, test.ShouldBeFalse)
	test.That(t, x.tcpLoadSource, test.ShouldEqual, tcpLoadSourceUnset)
}

func TestApplyTCPLoadAppliesOverRatingExplicitWrite(t *testing.T) {
	x := &xArm{logger: logging.NewTestLogger(t)}
	x.detectedArm = detectedArm{model: hardwareModelLite6}

	wrote := false
	err := x.applyTCPLoadWith(context.Background(), tcpLoad{massKg: 0.61},
		tcpLoadSourceDoCommand, "set_tcp_load",
		func(context.Context, tcpLoad) error { wrote = true; return nil })

	// The mirror of the refusal test above: the very same over-rating mass
	// reaches the controller and is cached when it arrives as an explicit
	// write rather than a pushed default — decideTCPLoad only warns, it never
	// blocks. This pins the asymmetry where it actually matters.
	test.That(t, err, test.ShouldBeNil)
	test.That(t, wrote, test.ShouldBeTrue)
	test.That(t, x.tcpLoadSource, test.ShouldEqual, tcpLoadSourceDoCommand)
	test.That(t, x.tcpLoad.massKg, test.ShouldAlmostEqual, 0.61, 1e-9)
}

// TestApplyTCPLoadIsAtomicAgainstConcurrentApply drives a deterministic
// interleaving of a gripper-default apply and a concurrent explicit apply
// through the injected-writer seam, rather than hoping a race turns up.
//
// The gripper's write callback parks on a channel so the test can hold it
// mid-apply — inside the critical section if tcpLoadApplyLock is held across
// the write, as it must be — while the explicit apply is launched. If the two
// applies are not serialized, the explicit apply's read of `current` can land
// while the gripper's is still in flight, both decide Apply, and whichever
// write reaches the controller last "wins" independent of cache order —
// exactly the mid-grasp corruption in the plan's motivating example. With
// serialization, the explicit apply cannot even start until the gripper's
// entire decide-write-cache sequence has completed, so it always sees the
// gripper's result as `current` and, per shouldApplyTCPLoad, still applies
// (explicit writes always do) — landing strictly after, in both the
// controller writes and the cache.
func TestApplyTCPLoadIsAtomicAgainstConcurrentApply(t *testing.T) {
	x := &xArm{logger: logging.NewTestLogger(t)}
	x.detectedArm = detectedArm{model: hardwareModelXArm6}

	var writesMu sync.Mutex
	var writes []float64

	gripperEntered := make(chan struct{})
	release := make(chan struct{})
	gripperDone := make(chan struct{})
	userDone := make(chan struct{})

	go func() {
		defer close(gripperDone)
		err := x.applyTCPLoadWith(context.Background(), tcpLoad{massKg: 0.61},
			tcpLoadSourceGripperDefault, "vacuum_gripper",
			func(context.Context, tcpLoad) error {
				close(gripperEntered)
				<-release
				// Widen the window a broken (unsynchronized) implementation would need
				// to let the concurrent explicit write interleave ahead of this one.
				time.Sleep(5 * time.Millisecond)
				writesMu.Lock()
				writes = append(writes, 0.61)
				writesMu.Unlock()
				return nil
			})
		test.That(t, err, test.ShouldBeNil)
	}()

	select {
	case <-gripperEntered:
	case <-time.After(2 * time.Second):
		t.Fatal("gripper goroutine never entered its write callback")
	}

	// Launched while the gripper's apply is still in flight (parked in its
	// write callback). If tcpLoadApplyLock serializes applies, this call
	// cannot proceed past Lock() until the gripper's entire apply — including
	// its cache update — has completed.
	go func() {
		defer close(userDone)
		err := x.applyTCPLoadWith(context.Background(), tcpLoad{massKg: 1.2},
			tcpLoadSourceDoCommand, "set_tcp_load",
			func(context.Context, tcpLoad) error {
				writesMu.Lock()
				writes = append(writes, 1.2)
				writesMu.Unlock()
				return nil
			})
		test.That(t, err, test.ShouldBeNil)
	}()

	close(release)

	select {
	case <-gripperDone:
	case <-time.After(2 * time.Second):
		t.Fatal("gripper goroutine never finished")
	}
	select {
	case <-userDone:
	case <-time.After(2 * time.Second):
		t.Fatal("user goroutine never finished")
	}

	writesMu.Lock()
	gotWrites := append([]float64(nil), writes...)
	writesMu.Unlock()

	// The explicit write must land last, both on the controller and in the
	// cache: they must agree, and the value must be the one a human actually
	// typed, not the stale default that raced in behind it.
	test.That(t, gotWrites, test.ShouldResemble, []float64{0.61, 1.2})

	x.confLock.Lock()
	defer x.confLock.Unlock()
	test.That(t, x.tcpLoadSource, test.ShouldEqual, tcpLoadSourceDoCommand)
	test.That(t, x.tcpLoad.massKg, test.ShouldAlmostEqual, 1.2, 1e-9)
}

func TestParseTCPLoadRequest(t *testing.T) {
	l, err := parseTCPLoadRequest(map[string]any{
		"mass_kg":              0.82,
		"center_of_gravity_mm": []any{1.0, 2.0, 48.0},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, l.massKg, test.ShouldAlmostEqual, 0.82, 1e-9)
	test.That(t, l.cogMM.X, test.ShouldAlmostEqual, 1, 1e-9)
	test.That(t, l.cogMM.Y, test.ShouldAlmostEqual, 2, 1e-9)
	test.That(t, l.cogMM.Z, test.ShouldAlmostEqual, 48, 1e-9)

	// CoG optional.
	l, err = parseTCPLoadRequest(map[string]any{"mass_kg": 1.0})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, l.cogMM, test.ShouldResemble, r3.Vector{})

	// Explicit zero mass is legal — it means "no payload".
	l, err = parseTCPLoadRequest(map[string]any{"mass_kg": 0.0})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, l.massKg, test.ShouldAlmostEqual, 0, 1e-9)

	// An explicit empty list agrees with omitting the key entirely: both mean
	// the origin. This must match toTCPLoad's reading of the same JSON shape
	// (see TestTCPLoadRequestAndConfigAgreeOnEmptyCOG).
	l, err = parseTCPLoadRequest(map[string]any{"mass_kg": 1.0, "center_of_gravity_mm": []any{}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, l.cogMM, test.ShouldResemble, r3.Vector{})

	// requester is a recognized key and must not be rejected by the allowlist.
	l, err = parseTCPLoadRequest(map[string]any{"mass_kg": 1.0, "requester": "vacuum_gripper"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, l.massKg, test.ShouldAlmostEqual, 1.0, 1e-9)

	// Rejections.
	for _, bad := range []map[string]any{
		{},                   // missing mass
		{"mass_kg": "heavy"}, // wrong type
		{"mass_kg": -1.0},    // negative
		{"mass_kg": 1.0, "center_of_gravity_mm": []any{1.0, 2.0}},      // short
		{"mass_kg": 1.0, "center_of_gravity_mm": []any{1.0, 2.0, "z"}}, // wrong element type
		// A misspelled optional key (missing "_mm") must be rejected outright,
		// not silently ignored: it is the only optional field, so a typo here
		// would otherwise parse cleanly and write a payload at the flange
		// origin instead of the center of gravity the caller intended.
		{"mass_kg": 1.0, "center_of_gravity": []any{0.0, 0.0, 48.0}},
		{"mass_kg": 1.0, "unexpected_key": true},
	} {
		_, err := parseTCPLoadRequest(bad)
		test.That(t, err, test.ShouldNotBeNil)
	}
}

// TestTCPLoadRequestAndConfigAgreeOnEmptyCOG pins that parseTCPLoadRequest
// (the DoCommand path) and toTCPLoad (the config path) give the same answer
// for an explicit empty center_of_gravity_mm: both treat it as the origin,
// same as omitting the key. They previously disagreed — toTCPLoad accepted
// it, parseTCPLoadRequest rejected it as "must have exactly 3 elements, got
// 0" — even though both parse the identical JSON shape.
func TestTCPLoadRequestAndConfigAgreeOnEmptyCOG(t *testing.T) {
	fromRequest, err := parseTCPLoadRequest(map[string]any{"mass_kg": 0.82, "center_of_gravity_mm": []any{}})
	test.That(t, err, test.ShouldBeNil)

	fromConfig, err := (&TCPLoadConfig{MassKg: massKgPtr(0.82), CenterOfGravityMM: []float64{}}).toTCPLoad()
	test.That(t, err, test.ShouldBeNil)

	test.That(t, fromRequest, test.ShouldResemble, fromConfig)
	test.That(t, fromRequest.cogMM, test.ShouldResemble, r3.Vector{})
}

func TestRequesterFor(t *testing.T) {
	// No requester param: the DoCommand key stands in.
	test.That(t, requesterFor("set_tcp_load", map[string]any{}), test.ShouldEqual, "set_tcp_load")
	test.That(t, requesterFor("set_default_tcp_load", nil), test.ShouldEqual, "set_default_tcp_load")

	// Explicit requester wins.
	test.That(t,
		requesterFor("set_default_tcp_load", map[string]any{"requester": "vacuum_gripper"}),
		test.ShouldEqual, "vacuum_gripper")

	// A present-but-empty or wrong-typed requester falls back to the key,
	// rather than caching an empty or garbage name.
	test.That(t, requesterFor("set_tcp_load", map[string]any{"requester": ""}), test.ShouldEqual, "set_tcp_load")
	test.That(t, requesterFor("set_tcp_load", map[string]any{"requester": 42}), test.ShouldEqual, "set_tcp_load")
}

func TestApplyTCPLoadCommand(t *testing.T) {
	t.Run("rejects a non-map value", func(t *testing.T) {
		x := &xArm{logger: logging.NewTestLogger(t)}
		err := x.applyTCPLoadCommand(context.Background(), setTCPLoadKey, "nope", tcpLoadSourceDoCommand)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, setTCPLoadKey)
	})

	t.Run("propagates parse errors, naming the DoCommand key", func(t *testing.T) {
		x := &xArm{logger: logging.NewTestLogger(t)}
		err := x.applyTCPLoadCommand(context.Background(), setTCPLoadKey,
			map[string]any{"mass_kg": -1.0}, tcpLoadSourceDoCommand)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, setTCPLoadKey)
		test.That(t, err.Error(), test.ShouldContainSubstring, "tcp load mass cannot be negative")
	})

	// Suppression lets this run to completion without a real controller
	// connection: decideTCPLoad returns before applyTCPLoadWith ever calls
	// the injected write function.
	t.Run("a suppressed default does not reach the controller", func(t *testing.T) {
		x := &xArm{logger: logging.NewTestLogger(t)}
		x.tcpLoad = tcpLoad{massKg: 1.2}
		x.tcpLoadSource = tcpLoadSourceDoCommand
		x.tcpLoadRequester = "set_tcp_load"

		err := x.applyTCPLoadCommand(context.Background(), setDefaultTCPLoadKey,
			map[string]any{"mass_kg": 0.61, "requester": "vacuum_gripper"}, tcpLoadSourceGripperDefault)
		test.That(t, err, test.ShouldBeNil)
		// Untouched: the earlier explicit write survives.
		test.That(t, x.tcpLoadSource, test.ShouldEqual, tcpLoadSourceDoCommand)
		test.That(t, x.tcpLoad.massKg, test.ShouldAlmostEqual, 1.2, 1e-9)
	})
}

func TestTCPLoadResponse(t *testing.T) {
	// Unset: numeric fields (and requester) must be omitted entirely.
	// Reporting 0 kg would be indistinguishable from a real zero payload.
	x := &xArm{logger: logging.NewTestLogger(t)}
	resp := x.tcpLoadResponse()
	test.That(t, resp["source"], test.ShouldEqual, "unset")
	_, hasMass := resp["mass_kg"]
	test.That(t, hasMass, test.ShouldBeFalse)
	_, hasCog := resp["center_of_gravity_mm"]
	test.That(t, hasCog, test.ShouldBeFalse)
	_, hasRequester := resp["requester"]
	test.That(t, hasRequester, test.ShouldBeFalse)

	// After a write, everything is reported, including who asked for it: a
	// gripper pushing set_default_tcp_load needs requester to tell its own
	// default apart from a competing gripper's that already won.
	x.tcpLoad = tcpLoad{massKg: 1.2, cogMM: r3.Vector{X: 1, Y: 2, Z: 48}}
	x.tcpLoadSource = tcpLoadSourceDoCommand
	x.tcpLoadRequester = "set_tcp_load"
	resp = x.tcpLoadResponse()
	test.That(t, resp["source"], test.ShouldEqual, "do_command")
	test.That(t, resp["mass_kg"], test.ShouldAlmostEqual, 1.2, 1e-9)
	test.That(t, resp["center_of_gravity_mm"], test.ShouldResemble, []float64{1, 2, 48})
	test.That(t, resp["requester"], test.ShouldEqual, "set_tcp_load")
}

// TestDoCommandTCPLoadKeys exercises set_tcp_load, set_default_tcp_load, and
// get_tcp_load through the real DoCommand entry point, against a bare *xArm
// with no socket. DoCommand has no connection-requiring preamble, so this
// works as long as every path taken either errors out before reaching the
// controller or is suppressed by precedence before applyTCPLoadWith calls
// its injected write function — a real write would panic on the nil
// connection. That constraint is what makes the last case below valuable: it
// exercises the full set_default_tcp_load block, including which
// tcpLoadSource it passes to applyTCPLoad, without needing a socket. Pin
// wrong there (e.g. swapped with tcpLoadSourceDoCommand) and suppression
// stops firing, the call reaches the controller, and the whole test binary
// panics.
func TestDoCommandTCPLoadKeys(t *testing.T) {
	t.Run("get_tcp_load when unset", func(t *testing.T) {
		x := &xArm{logger: logging.NewTestLogger(t)}
		resp, err := x.DoCommand(context.Background(), map[string]any{getTCPLoadKey: true})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, resp, test.ShouldResemble, map[string]any{
			tcpLoadKey: map[string]any{"source": "unset"},
		})
	})

	t.Run("set_tcp_load rejects a non-map value, naming the key and the type", func(t *testing.T) {
		x := &xArm{logger: logging.NewTestLogger(t)}
		_, err := x.DoCommand(context.Background(), map[string]any{setTCPLoadKey: "nope"})
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, setTCPLoadKey)
		test.That(t, err.Error(), test.ShouldContainSubstring, "string")
	})

	t.Run("set_tcp_load rejects an invalid payload before touching the controller", func(t *testing.T) {
		x := &xArm{logger: logging.NewTestLogger(t)}
		_, err := x.DoCommand(context.Background(), map[string]any{
			setTCPLoadKey: map[string]any{"mass_kg": -1.0},
		})
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "tcp load mass cannot be negative")
	})

	t.Run("set_default_tcp_load suppressed by a prior explicit write reports the retained payload", func(t *testing.T) {
		x := &xArm{logger: logging.NewTestLogger(t)}
		x.tcpLoad = tcpLoad{massKg: 1.2}
		x.tcpLoadSource = tcpLoadSourceDoCommand
		x.tcpLoadRequester = "set_tcp_load"

		resp, err := x.DoCommand(context.Background(), map[string]any{
			setDefaultTCPLoadKey: map[string]any{"mass_kg": 0.61, "requester": "vacuum_gripper"},
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, resp[tcpLoadKey], test.ShouldResemble, map[string]any{
			"source":               "do_command",
			"mass_kg":              1.2,
			"center_of_gravity_mm": []float64{0, 0, 0},
			"requester":            "set_tcp_load",
		})
	})
}

func TestPushGripperDefaultSkipsUnknownModel(t *testing.T) {
	// A model with no preset must not produce a DoCommand at all.
	called := false
	fake := &tcpLoadRecorderArm{onDo: func(map[string]any) { called = true }}

	pushGripperDefaultTCPLoad(context.Background(), fake, GripperModelLite, logging.NewTestLogger(t))
	test.That(t, called, test.ShouldBeFalse)

	pushGripperDefaultTCPLoad(context.Background(), fake, VacuumGripperModelLite, logging.NewTestLogger(t))
	test.That(t, called, test.ShouldBeFalse)
}

func TestPushGripperDefaultSendsPreset(t *testing.T) {
	for _, tc := range []struct {
		model   resource.Model
		wantKg  float64
		wantCog []any
	}{
		{GripperModel, 0.82, []any{0.0, 0.0, 48.0}},
		{VacuumGripperModel, 0.61, []any{0.0, 0.0, 53.0}},
	} {
		t.Run(tc.model.String(), func(t *testing.T) {
			var got map[string]any
			fake := &tcpLoadRecorderArm{onDo: func(cmd map[string]any) { got = cmd }}

			pushGripperDefaultTCPLoad(context.Background(), fake, tc.model, logging.NewTestLogger(t))

			params, ok := got[setDefaultTCPLoadKey].(map[string]any)
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, params["mass_kg"], test.ShouldAlmostEqual, tc.wantKg, 1e-9)
			test.That(t, params["center_of_gravity_mm"], test.ShouldResemble, tc.wantCog)
			test.That(t, params["requester"], test.ShouldEqual, tc.model.String())
		})
	}
}

// The pushed map must survive the real parser — otherwise a shape mismatch
// between the producer and consumer only surfaces on hardware.
func TestPushedGripperDefaultParses(t *testing.T) {
	for _, model := range []resource.Model{GripperModel, VacuumGripperModel} {
		var got map[string]any
		fake := &tcpLoadRecorderArm{onDo: func(cmd map[string]any) { got = cmd }}
		pushGripperDefaultTCPLoad(context.Background(), fake, model, logging.NewTestLogger(t))

		params := got[setDefaultTCPLoadKey].(map[string]any)
		parsed, err := parseTCPLoadRequest(params)
		test.That(t, err, test.ShouldBeNil)

		want, ok := gripperDefaultTCPLoad(model)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, parsed, test.ShouldResemble, want)
	}
}
