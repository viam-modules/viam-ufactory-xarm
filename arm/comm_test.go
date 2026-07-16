package arm

import (
	"encoding/binary"
	"math"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
)

func TestParseFTSensorData(t *testing.T) {
	// Build a controller frame: 1 leading byte, then 6 little-endian float32 values.
	want := []float64{-0.9871726, -2.9230627, -18.356257, -0.0012335, -0.0913722, 0.0069847}
	params := make([]byte, 1+6*4)
	for i, v := range want {
		binary.LittleEndian.PutUint32(params[i*4+1:i*4+5], math.Float32bits(float32(v)))
	}

	got, err := parseFTSensorData(params)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(got), test.ShouldEqual, 6)
	for i := range want {
		test.That(t, got[i], test.ShouldAlmostEqual, want[i], 1e-4)
	}

	_, err = parseFTSensorData(make([]byte, 4))
	test.That(t, err, test.ShouldNotBeNil)
}

func TestTrajectoryStreamValidator(t *testing.T) {
	// point builds a trajectory point at time `d` with `dof` zeroed joint positions.
	point := func(d time.Duration, dof int) arm.TrajectoryPoint {
		return arm.TrajectoryPoint{Time: d, Positions: make([]referenceframe.Input, dof)}
	}
	// pointWithVel is `point` plus a declared velocity on each joint.
	pointWithVel := func(d time.Duration, dof int, vel float64) arm.TrajectoryPoint {
		p := point(d, dof)
		vels := make([]float64, dof)
		for i := range vels {
			vels[i] = vel
		}
		p.Constraints = &arm.KinematicConstraints{Velocities: vels}
		return p
	}

	// The validator carries state across a stream, so each case feeds a whole sequence and asserts
	// where, if anywhere, the first rejection lands. A `failAtIdx` of -1 means the sequence is valid.
	for _, tc := range []struct {
		name      string
		points    []arm.TrajectoryPoint
		failAtIdx int
	}{
		{"valid increasing stream", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 6), point(20*time.Millisecond, 6)}, -1},
		{"first point must be t=0", []arm.TrajectoryPoint{point(5*time.Millisecond, 6)}, 0},
		{"time must strictly increase", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 6), point(10*time.Millisecond, 6)}, 2},
		{"time may not move backwards", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 6), point(5*time.Millisecond, 6)}, 2},
		{"dof must stay consistent", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 7)}, 1},
		{"positions must be non-empty", []arm.TrajectoryPoint{point(0, 0)}, 0},
		{"first point must be at rest", []arm.TrajectoryPoint{pointWithVel(0, 6, 0.5)}, 0},
		{"first point at rest with zero velocity is fine", []arm.TrajectoryPoint{pointWithVel(0, 6, 0), pointWithVel(10*time.Millisecond, 6, 0.5)}, -1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			v := newTrajectoryStreamValidator()
			firstErrIdx := -1
			for i, pt := range tc.points {
				if err := v.validate(pt); err != nil {
					firstErrIdx = i
					break
				}
			}
			test.That(t, firstErrIdx, test.ShouldEqual, tc.failAtIdx)
		})
	}
}

func TestCreateRawJointSteps1(t *testing.T) {
	var err error
	logger := logging.NewTestLogger(t)

	x := &xArm{
		speed:        utils.DegToRad(defaultSpeed),
		acceleration: utils.DegToRad(defaultAccel),
		moveHZ:       defaultMoveHz,
	}

	start := []float64{0, 0, 0, 0, 0, 0}
	x.model, err = MakeModelFrame("", ModelName6DOF, nil, start, false, nil, logger)
	test.That(t, err, test.ShouldBeNil)

	positions := [][]float64{{1, 0, 0, 0, 0, 1}}

	out, err := x.createRawJointSteps(start, positions, x.moveOptions(nil, nil))
	test.That(t, err, test.ShouldBeNil)

	minMoves := (1 / x.speed) * x.moveHZ
	test.That(t, len(out), test.ShouldBeGreaterThan, minMoves)
	test.That(t, len(out), test.ShouldBeLessThan, 20+minMoves)
}

func TestCreateRawJointStepsLowSpeed(t *testing.T) {
	var err error
	logger := logging.NewTestLogger(t)

	x := &xArm{
		speed:        utils.DegToRad(3),
		acceleration: utils.DegToRad(defaultAccel),
		moveHZ:       defaultMoveHz,
	}

	start := []float64{0, 0, 0, 0, 0, 0}
	x.model, err = MakeModelFrame("", ModelName6DOF, nil, start, false, nil, logger)
	test.That(t, err, test.ShouldBeNil)

	displacement := 1.0
	positions := [][]float64{{displacement, 0, 0, 0, 0, displacement}}

	out, err := x.createRawJointSteps(start, positions, x.moveOptions(nil, nil))
	test.That(t, err, test.ShouldBeNil)

	expected := (displacement / x.speed) * x.moveHZ
	// 15% band absorbs accel/decel ramp overhead.
	test.That(t, float64(len(out)), test.ShouldBeGreaterThan, 0.85*expected)
	test.That(t, float64(len(out)), test.ShouldBeLessThan, 1.15*expected)
}
