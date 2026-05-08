package arm

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestXArmGripperKinematicsModel(t *testing.T) {
	model, err := referenceframe.UnmarshalModelJSON(xArmGripperModelJSON, "xarm-gripper")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, model.Name(), test.ShouldEqual, "xarm-gripper")

	// One DoF — the right finger mimics the left.
	test.That(t, len(model.DoF()), test.ShouldEqual, 1)
	test.That(t, model.DoF()[0].Min, test.ShouldEqual, 0)
	test.That(t, model.DoF()[0].Max, test.ShouldEqual, gripperHalfStrokeMM)
}

func TestXArmGripperFingerSymmetry(t *testing.T) {
	model, err := referenceframe.UnmarshalModelJSON(xArmGripperModelJSON, "xarm-gripper")
	test.That(t, err, test.ShouldBeNil)

	// At fully-open, both finger inner faces should be at ±gripperHalfStrokeMM.
	gif, err := model.Geometries([]referenceframe.Input{gripperHalfStrokeMM})
	test.That(t, err, test.ShouldBeNil)

	left := gif.GeometryByName("xarm-gripper:left_finger").Pose().Point()
	right := gif.GeometryByName("xarm-gripper:right_finger").Pose().Point()

	// Left finger geometry center sits at half_stroke + finger_thickness/2 in +Y.
	// Right is the mirror.
	test.That(t, spatialmath.R3VectorAlmostEqual(
		r3.Vector{X: left.X, Y: -left.Y, Z: left.Z}, right, 1e-6,
	), test.ShouldBeTrue)

	// The TCP is a static link at z = case_height + finger_length = 160.
	tcpPose, err := model.Transform([]referenceframe.Input{gripperHalfStrokeMM})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.R3VectorAlmostEqual(
		tcpPose.Point(), r3.Vector{X: 0, Y: 0, Z: 160}, 1e-6,
	), test.ShouldBeTrue)
}

func TestGripperHardwareMMConversion(t *testing.T) {
	// Hardware 0 → 0 mm; hardware 850 → 42.5 mm; round-trip stable.
	test.That(t, hardwarePositionToFingerMM(0), test.ShouldEqual, 0.0)
	test.That(t, hardwarePositionToFingerMM(840), test.ShouldEqual, gripperHalfStrokeMM)
	test.That(t, fingerMMToHardwarePosition(0), test.ShouldEqual, 0)
	test.That(t, fingerMMToHardwarePosition(gripperHalfStrokeMM), test.ShouldEqual, 840)

	// Out-of-range inputs are clamped.
	test.That(t, hardwarePositionToFingerMM(-50), test.ShouldEqual, 0.0)
	test.That(t, hardwarePositionToFingerMM(2000), test.ShouldEqual, gripperHalfStrokeMM)
	test.That(t, fingerMMToHardwarePosition(-1), test.ShouldEqual, 0)
	test.That(t, fingerMMToHardwarePosition(100), test.ShouldEqual, gripperHardwareMax)

	// Mid-range round-trip.
	test.That(t, hardwarePositionToFingerMM(420), test.ShouldEqual, 21.0)
	test.That(t, fingerMMToHardwarePosition(21.0), test.ShouldEqual, 420)
}
