package arm

import (
	"context"
	"errors"
	"testing"

	"go.viam.com/test"
)

func TestGripperConfigValidate(t *testing.T) {
	t.Run("requires arm", func(t *testing.T) {
		cfg := &GripperConfig{}
		_, _, err := cfg.Validate("path")
		test.That(t, err, test.ShouldNotBeNil)
	})

	t.Run("validates speed range", func(t *testing.T) {
		cfg := &GripperConfig{Arm: "xarm", GripperSpeed: 5001}
		_, _, err := cfg.Validate("path")
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_speed must be between 1 and 5000")
	})

	t.Run("validates force range", func(t *testing.T) {
		cfg := &GripperConfig{Arm: "xarm", GripperForce: 101}
		_, _, err := cfg.Validate("path")
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_force must be between 1 and 100")
	})

	t.Run("valid config", func(t *testing.T) {
		cfg := &GripperConfig{Arm: "xarm", GripperSpeed: 2000, GripperForce: 50}
		deps, _, err := cfg.Validate("path")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, deps, test.ShouldResemble, []string{"xarm"})
	})
}

func TestLegacyGripperRejectsForceDoCommand(t *testing.T) {
	g := &myGripper{supportsForce: false}

	_, err := g.DoCommand(context.Background(), map[string]any{setGripperForceKey: 50.0})
	test.That(t, err, test.ShouldEqual, errors.ErrUnsupported)

	_, err = g.DoCommand(context.Background(), map[string]any{getGripperForceKey: true})
	test.That(t, err, test.ShouldEqual, errors.ErrUnsupported)
}
