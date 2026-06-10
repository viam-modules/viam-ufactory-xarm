package arm

import (
	"encoding/binary"
	"math"
	"testing"

	"go.viam.com/rdk/logging"
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

func TestCreateRawJointSteps1(t *testing.T) {
	var err error
	logger := logging.NewTestLogger(t)

	x := &xArm{
		speed:        utils.DegToRad(defaultSpeed),
		acceleration: utils.DegToRad(defaultAccel),
		moveHZ:       defaultMoveHz,
	}

	start := []float64{0, 0, 0, 0, 0, 0}
	x.model, err = MakeModelFrame("", ModelName6DOF, nil, start, false, nil, logger, 0)
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
	x.model, err = MakeModelFrame("", ModelName6DOF, nil, start, false, nil, logger, 0)
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

func TestTgpioWord(t *testing.T) {
	test.That(t, tgpioWord(0, true), test.ShouldEqual, uint16(0x0101))  // v1 ON  pin0
	test.That(t, tgpioWord(1, false), test.ShouldEqual, uint16(0x0200)) // v1 ON  pin1
	test.That(t, tgpioWord(0, false), test.ShouldEqual, uint16(0x0100)) // v1 OFF pin0
	test.That(t, tgpioWord(1, true), test.ShouldEqual, uint16(0x0202))  // v1 OFF pin1
	test.That(t, tgpioWord(3, true), test.ShouldEqual, uint16(0x0404))  // v2 ON  pin3
	test.That(t, tgpioWord(4, false), test.ShouldEqual, uint16(0x0800)) // v2 ON  pin4
	test.That(t, tgpioWord(3, false), test.ShouldEqual, uint16(0x0400)) // v2 OFF pin3
	test.That(t, tgpioWord(4, true), test.ShouldEqual, uint16(0x0808))  // v2 OFF pin4
}

func TestTgpioDigitalParams_V1Regression(t *testing.T) {
	test.That(t, tgpioDigitalParams(0, true), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x80, 0x80, 0x43})
	test.That(t, tgpioDigitalParams(1, false), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x00, 0x00, 0x44})
	test.That(t, tgpioDigitalParams(0, false), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x00, 0x80, 0x43})
	test.That(t, tgpioDigitalParams(1, true), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x80, 0x00, 0x44})
}

func TestVacuumStateFromResponse(t *testing.T) {
	holdingV1 := []byte{0, 0, 0, 0, 0x01}
	holdingV2 := []byte{0, 0, 0, 0, 0x04}
	idle := []byte{0, 0, 0, 0, 0x00}

	got, err := vacuumStateFromResponse(holdingV1, connectionPlugin)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeTrue)

	got, err = vacuumStateFromResponse(holdingV2, connectionContact)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeTrue)

	got, err = vacuumStateFromResponse(holdingV1, connectionContact)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeFalse)

	got, err = vacuumStateFromResponse(idle, connectionPlugin)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeFalse)

	// Extra high bits set: masking (not equality) still reads as holding.
	got, err = vacuumStateFromResponse([]byte{0, 0, 0, 0, 0x05}, connectionPlugin)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeTrue)

	_, err = vacuumStateFromResponse([]byte{0, 0}, connectionPlugin)
	test.That(t, err, test.ShouldNotBeNil)
}
