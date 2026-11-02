package arm

import (
	"testing"

	"go.viam.com/test"
)

func TestGripperConfigValidateConnectionType(t *testing.T) {
	base := func(ct string) *GripperConfig {
		return &GripperConfig{Arm: "a", ConnectionType: ct}
	}
	for _, ok := range []string{"", "plugin", "contact"} {
		_, _, err := base(ok).Validate("p")
		test.That(t, err, test.ShouldBeNil)
	}
	_, _, err := base("wired").Validate("p")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "connection_type")
}

func TestGripperConfigValidateVersionAndForce(t *testing.T) {
	_, _, err := (&GripperConfig{Arm: "a", GripperVersion: "G3"}).Validate("p")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_version")

	_, _, err = (&GripperConfig{Arm: "a", GripperForce: 101}).Validate("p")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_force")
}

func TestGripperConfigValidateVacuumSealTimeout(t *testing.T) {
	_, _, err := (&GripperConfig{Arm: "a", VacuumSealTimeoutMS: 500}).Validate("p")
	test.That(t, err, test.ShouldBeNil)

	_, _, err = (&GripperConfig{Arm: "a", VacuumSealTimeoutMS: -1}).Validate("p")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "vacuum_seal_timeout_ms")
}

func TestAtMoveG2Goal(t *testing.T) {
	for _, tc := range []struct {
		name   string
		status uint16
		pos    int
		goal   int
		want   bool
	}{
		// Open on already-open jaws: stopped at ~838 for a goal of 840 -> done.
		{"open already open", gripperStateStop, 838, fullyOpenPosition, true},
		{"open just under tolerance", gripperStateStop, fullyOpenPosition - gripperGoalTolerance, fullyOpenPosition, true},
		// Open from closed: stopped near 0 but far from 840 -> must move.
		{"open from closed", gripperStateStop, 2, fullyOpenPosition, false},
		// Grab on jaws already closed on nothing: stopped near 0 for a goal of 0 -> done.
		{"grab already closed on nothing", gripperStateStop, -1, 0, true},
		// Grab holding an object: object-detected keeps the full write+wait path.
		{"grab object detected at goal", gripperStateDetected, 0, 0, false},
		// Mid-move and fault never short-circuit.
		{"in motion", gripperStateMotion, fullyOpenPosition, fullyOpenPosition, false},
		{"fault at goal", gripperStateFault, fullyOpenPosition, fullyOpenPosition, false},
		// Just outside tolerance still moves.
		{"just outside tolerance", gripperStateStop, fullyOpenPosition - gripperGoalTolerance - 1, fullyOpenPosition, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			test.That(t, atMoveG2Goal(tc.status, tc.pos, tc.goal), test.ShouldEqual, tc.want)
		})
	}
}
