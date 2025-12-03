package arm

import (
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
	x.model, err = MakeModelFrame(ModelName6DOF, nil, start, logger)
	test.That(t, err, test.ShouldBeNil)

	positions := [][]float64{{1, 0, 0, 0, 0, 1}}

	out, err := x.createRawJointSteps(start, positions, nil)
	test.That(t, err, test.ShouldBeNil)

	minMoves := (1 / x.speed) * x.moveHZ
	test.That(t, len(out), test.ShouldBeGreaterThan, minMoves)
	test.That(t, len(out), test.ShouldBeLessThan, 20+minMoves)
}
