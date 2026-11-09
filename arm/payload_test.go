package arm

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.uber.org/zap/zapcore"
	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

func TestDecideTCPPayload(t *testing.T) {
	preset := &presetGripperG2
	unset := &tcpPayload{}
	// The report carries float32 values, so a stored preset reads back slightly off.
	roundTripped := &tcpPayload{MassKg: float64(float32(0.82)), CenterOfGravityMM: r3.Vector{Z: 48}}
	withinTolerance := &tcpPayload{MassKg: 0.86, CenterOfGravityMM: r3.Vector{Z: 50}}
	heavy := &tcpPayload{MassKg: 4.0, CenterOfGravityMM: r3.Vector{Z: 60}}
	light := &tcpPayload{MassKg: 0.3, CenterOfGravityMM: r3.Vector{Z: 20}}

	for _, tc := range []struct {
		name     string
		reported *tcpPayload
		preset   *gripperPreset
		want     *tcpPayload
		wantInfo string
		wantWarn string
	}{
		{name: "unset controller gets preset", reported: unset, preset: preset, want: &preset.payload, wantInfo: "applying the xArm Gripper G2 preset"},
		{name: "unset controller without gripper", reported: unset, wantInfo: "assumes nothing is mounted"},
		{name: "preset already on controller", reported: roundTripped, preset: preset, wantInfo: "matches the xArm Gripper G2 preset"},
		{name: "close to preset counts as preset", reported: withinTolerance, preset: preset, wantInfo: "matches the xArm Gripper G2 preset"},
		{name: "heavy Studio payload kept with warning", reported: heavy, preset: preset, wantWarn: "mass=4.000kg"},
		{name: "light Studio payload kept with warning", reported: light, preset: preset, wantWarn: "the xArm Gripper G2 preset is 0.820kg"},
		{name: "payload without gripper kept with warning", reported: heavy, wantWarn: "no UFACTORY gripper is registered"},
		{name: "unreadable controller left alone", preset: preset, wantWarn: "unknown"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			logger, logs := logging.NewObservedTestLogger(t)
			got := decideTCPPayload(tc.reported, tc.preset, logger)
			test.That(t, got, test.ShouldEqual, tc.want)

			if tc.wantInfo != "" {
				test.That(t, logs.FilterLevelExact(zapcore.InfoLevel).FilterMessageSnippet(tc.wantInfo).Len(), test.ShouldEqual, 1)
			}
			warns := logs.FilterLevelExact(zapcore.WarnLevel)
			if tc.wantWarn == "" {
				test.That(t, warns.Len(), test.ShouldEqual, 0)
			} else {
				test.That(t, warns.FilterMessageSnippet(tc.wantWarn).Len(), test.ShouldEqual, 1)
				test.That(t, warns.FilterMessageSnippet("UFACTORY Studio").Len()+warns.FilterMessageSnippet("unknown").Len(),
					test.ShouldEqual, 1)
			}
		})
	}
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
		test.That(t, got.payload.MassKg, test.ShouldBeGreaterThan, 0)
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
