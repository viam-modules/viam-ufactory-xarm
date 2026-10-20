package arm

import (
	"encoding/json"
	"testing"

	"github.com/golang/geo/r3"
	"go.uber.org/zap/zapcore"
	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

func TestTCPPayloadConfigJSON(t *testing.T) {
	var p TCPPayloadConfig
	err := json.Unmarshal([]byte(`{"mass_kg": 0.82, "center_of_gravity_mm": {"x": 1, "y": -2, "z": 48}}`), &p)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, p, test.ShouldResemble, TCPPayloadConfig{MassKg: 0.82, CenterOfGravityMM: r3.Vector{X: 1, Y: -2, Z: 48}})
}

func TestDecideTCPPayload(t *testing.T) {
	configured := &TCPPayloadConfig{MassKg: 1.2, CenterOfGravityMM: r3.Vector{Z: 70}}
	light := &TCPPayloadConfig{MassKg: 0.3, CenterOfGravityMM: r3.Vector{Z: 20}}
	heavy := &TCPPayloadConfig{MassKg: 1.0, CenterOfGravityMM: r3.Vector{Z: 60}}
	unset := &TCPPayloadConfig{}
	preset := &presetGripperG2

	for _, tc := range []struct {
		name       string
		configured *TCPPayloadConfig
		reported   *TCPPayloadConfig
		preset     *gripperPreset
		want       *TCPPayloadConfig
		wantWarn   string
	}{
		{name: "config wins over controller", configured: configured, reported: heavy, preset: preset, want: configured},
		{name: "config wins when report unreadable", configured: configured, preset: preset, want: configured},
		{name: "light config warns", configured: light, reported: unset, preset: preset, want: light, wantWarn: "lighter than the xArm Gripper G2"},
		{name: "controller payload kept", reported: heavy, preset: preset},
		{name: "light controller payload kept with warning", reported: light, preset: preset, wantWarn: "lighter than the xArm Gripper G2"},
		{name: "unset controller gets preset", reported: unset, preset: preset, want: &preset.payload},
		{name: "unset controller without gripper left alone", reported: unset},
		{name: "unreadable controller left alone", preset: preset, wantWarn: "unknown"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			logger, logs := logging.NewObservedTestLogger(t)
			got := decideTCPPayload(tc.configured, tc.reported, tc.preset, logger)
			test.That(t, got, test.ShouldEqual, tc.want)

			warns := logs.FilterLevelExact(zapcore.WarnLevel)
			if tc.wantWarn == "" {
				test.That(t, warns.Len(), test.ShouldEqual, 0)
			} else {
				test.That(t, warns.FilterMessageSnippet(tc.wantWarn).Len(), test.ShouldEqual, 1)
			}
		})
	}
}

func TestPayloadMatches(t *testing.T) {
	p := &TCPPayloadConfig{MassKg: 0.82, CenterOfGravityMM: r3.Vector{Z: 48}}
	// The report carries float32, so a round trip must still match.
	roundTripped := &TCPPayloadConfig{MassKg: float64(float32(0.82)), CenterOfGravityMM: r3.Vector{Z: 48}}
	test.That(t, payloadMatches(p, roundTripped), test.ShouldBeTrue)
	test.That(t, payloadMatches(p, nil), test.ShouldBeFalse)
	test.That(t, payloadMatches(p, &TCPPayloadConfig{MassKg: 0.83, CenterOfGravityMM: r3.Vector{Z: 48}}), test.ShouldBeFalse)
	test.That(t, payloadMatches(p, &TCPPayloadConfig{MassKg: 0.82, CenterOfGravityMM: r3.Vector{Z: 50}}), test.ShouldBeFalse)
}

func TestPresetFor(t *testing.T) {
	for _, tc := range []struct {
		model    string
		submodel string
		want     gripperPreset
	}{
		{ModelNameGripper, submodelG2, presetGripperG2},
		{ModelNameGripper, submodelG1, presetGripperG1},
		{ModelNameGripper, "", presetGripperG1},
		{ModelNameVacuumGripper, "", presetVacuumGripper},
		{ModelNameGripperLite, "", presetGripperLite},
		{ModelNameVacuumGripperLite, "", presetVacuumGripperLite},
	} {
		got, ok := presetFor(tc.model, detectedGripper{submodel: tc.submodel})
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, got, test.ShouldResemble, tc.want)
		test.That(t, got.payload.validate(), test.ShouldBeNil)
	}

	_, ok := presetFor("not-a-gripper", detectedGripper{})
	test.That(t, ok, test.ShouldBeFalse)
}

func TestRegisterGripperPreset(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("registers and releases", func(t *testing.T) {
		x := &xArm{conf: &Config{}, logger: logger}
		x.payloadChecked.Store(true)

		release := registerGripperPreset(x, ModelNameGripper, detectedGripper{submodel: submodelG2}, logger)
		test.That(t, *x.gripperPreset.Load(), test.ShouldResemble, presetGripperG2)
		test.That(t, x.payloadChecked.Load(), test.ShouldBeFalse)

		release()
		test.That(t, x.gripperPreset.Load(), test.ShouldBeNil)
	})

	t.Run("releasing an old gripper keeps a newer gripper's preset", func(t *testing.T) {
		x := &xArm{conf: &Config{}, logger: logger}

		releaseOld := registerGripperPreset(x, ModelNameGripper, detectedGripper{}, logger)
		releaseNew := registerGripperPreset(x, ModelNameVacuumGripper, detectedGripper{}, logger)
		releaseOld()
		test.That(t, *x.gripperPreset.Load(), test.ShouldResemble, presetVacuumGripper)
		releaseNew()
		test.That(t, x.gripperPreset.Load(), test.ShouldBeNil)
	})
}
