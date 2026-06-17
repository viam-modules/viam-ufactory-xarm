package arm

import (
	"context"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
)

func TestCreateRawJointSteps1(t *testing.T) {
	var err error
	logger := logging.NewTestLogger(t)

	x := &xArm{
		speed:        utils.DegToRad(defaultSpeed),
		acceleration: utils.DegToRad(defaultAccel),
		moveHZ:       defaultMoveHz,
	}

	start := []float64{0, 0, 0, 0, 0, 0}
	x.model, err = MakeModelFrame(ModelName6DOF, nil, start, false, nil, logger)
	test.That(t, err, test.ShouldBeNil)

	positions := [][]float64{{1, 0, 0, 0, 0, 1}}

	out, err := x.createRawJointSteps(start, positions, x.moveOptions(nil, nil))
	test.That(t, err, test.ShouldBeNil)

	minMoves := (1 / x.speed) * x.moveHZ
	test.That(t, len(out), test.ShouldBeGreaterThan, minMoves)
	test.That(t, len(out), test.ShouldBeLessThan, 20+minMoves)
}

func TestParseSensitivityOverride(t *testing.T) {
	// Absent key: no override, no error.
	level, ok, err := parseSensitivityOverride(map[string]any{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeFalse)
	test.That(t, level, test.ShouldEqual, 0)

	// Nil extra behaves like absent.
	_, ok, err = parseSensitivityOverride(nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, ok, test.ShouldBeFalse)

	// Valid values across the whole range (JSON numbers arrive as float64).
	for _, v := range []float64{0, 1, 3, 5} {
		level, ok, err = parseSensitivityOverride(map[string]any{collisionSensitivityKey: v})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, level, test.ShouldEqual, int(v))
	}

	// Out of range.
	_, _, err = parseSensitivityOverride(map[string]any{collisionSensitivityKey: 6.0})
	test.That(t, err, test.ShouldNotBeNil)
	_, _, err = parseSensitivityOverride(map[string]any{collisionSensitivityKey: -1.0})
	test.That(t, err, test.ShouldNotBeNil)

	// Non-integer.
	_, _, err = parseSensitivityOverride(map[string]any{collisionSensitivityKey: 2.5})
	test.That(t, err, test.ShouldNotBeNil)

	// Wrong type.
	_, _, err = parseSensitivityOverride(map[string]any{collisionSensitivityKey: "3"})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestApplyCollisionSensitivityOverrideNoHardware(t *testing.T) {
	x := &xArm{conf: &Config{}}

	// No override present: restore is a no-op that touches no hardware and
	// reports no error.
	restore, err := x.applyCollisionSensitivityOverride(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	moveErr := error(nil)
	restore(&moveErr)
	test.That(t, moveErr, test.ShouldBeNil)

	// Malformed override: error is returned before any hardware command, and
	// the returned restore is still safe to call.
	restore, err = x.applyCollisionSensitivityOverride(context.Background(), map[string]any{collisionSensitivityKey: "bad"})
	test.That(t, err, test.ShouldNotBeNil)
	moveErr = nil
	restore(&moveErr)
	test.That(t, moveErr, test.ShouldBeNil)
}

func TestBaselineCollisionSensitivity(t *testing.T) {
	// No configured value falls back to the firmware default.
	x := &xArm{conf: &Config{}}
	test.That(t, x.baselineCollisionSensitivity(), test.ShouldEqual, defaultCollisionSensitivity)

	// Configured value (including 0 = off) is honored.
	for _, v := range []int{0, 2, 5} {
		val := v
		x := &xArm{conf: &Config{Sensitivity: &val}}
		test.That(t, x.baselineCollisionSensitivity(), test.ShouldEqual, v)
	}
}
