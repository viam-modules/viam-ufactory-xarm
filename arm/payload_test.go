package arm

import (
	"encoding/json"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

func TestTCPPayloadConfigJSON(t *testing.T) {
	var p TCPPayloadConfig
	err := json.Unmarshal([]byte(`{"mass_kg": 0.82, "center_of_gravity_mm": {"x": 1, "y": -2, "z": 48}}`), &p)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, p, test.ShouldResemble, TCPPayloadConfig{MassKg: 0.82, CenterOfGravityMM: r3.Vector{X: 1, Y: -2, Z: 48}})
}

func TestClaimGripperPayload(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("gripper default applies when arm has none", func(t *testing.T) {
		x := &xArm{conf: &Config{}, logger: logger}
		x.tcpPayloadApplied.Store(true)

		release := claimGripperPayload(x, ModelNameGripper, logger)
		p := x.effectiveTCPPayload()
		test.That(t, p, test.ShouldNotBeNil)
		test.That(t, *p, test.ShouldResemble, defaultGripperPayloads[ModelNameGripper])
		test.That(t, x.tcpPayloadApplied.Load(), test.ShouldBeFalse)

		release()
		test.That(t, x.effectiveTCPPayload(), test.ShouldBeNil)
	})

	t.Run("arm tcp_payload wins over gripper default", func(t *testing.T) {
		armPayload := &TCPPayloadConfig{MassKg: 1.2, CenterOfGravityMM: r3.Vector{Z: 70}}
		x := &xArm{conf: &Config{TCPPayload: armPayload}, logger: logger}

		release := claimGripperPayload(x, ModelNameVacuumGripper, logger)
		test.That(t, x.effectiveTCPPayload(), test.ShouldEqual, armPayload)
		test.That(t, x.gripperPayload.Load(), test.ShouldBeNil)
		release()
		test.That(t, x.effectiveTCPPayload(), test.ShouldEqual, armPayload)
	})

	t.Run("releasing an old gripper keeps a newer gripper's payload", func(t *testing.T) {
		x := &xArm{conf: &Config{}, logger: logger}

		releaseOld := claimGripperPayload(x, ModelNameGripper, logger)
		releaseNew := claimGripperPayload(x, ModelNameVacuumGripper, logger)
		releaseOld()
		test.That(t, *x.effectiveTCPPayload(), test.ShouldResemble, defaultGripperPayloads[ModelNameVacuumGripper])
		releaseNew()
		test.That(t, x.effectiveTCPPayload(), test.ShouldBeNil)
	})

	t.Run("every gripper model has a valid default", func(t *testing.T) {
		for _, m := range []string{ModelNameGripper, ModelNameGripperLite, ModelNameVacuumGripper, ModelNameVacuumGripperLite} {
			p, ok := defaultGripperPayloads[m]
			test.That(t, ok, test.ShouldBeTrue)
			test.That(t, p.validate(), test.ShouldBeNil)
		}
	})
}
