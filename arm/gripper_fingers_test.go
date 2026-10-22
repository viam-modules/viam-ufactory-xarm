package arm

import (
	"context"
	"fmt"
	"math"
	"path/filepath"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func loadFingerModel(t *testing.T) referenceframe.Model {
	t.Helper()
	t.Setenv("VIAM_MODULE_ROOT", filepath.Dir(armDir()))
	mf, err := loadStandardGripperFingerModel(ModelNameGripper, nil, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	return mf
}

// leftFingerOrigin is where the left_finger link sits in the base frame at drive
// angle q: the outer knuckle pivot plus the 55 mm knuckle vector rotated about +X.
func leftFingerOrigin(q float64) r3.Vector {
	return r3.Vector{
		Y: 35 + 35.465*math.Cos(q) - 42.039*math.Sin(q),
		Z: 59.098 + 35.465*math.Sin(q) + 42.039*math.Cos(q),
	}
}

func geometriesByLabel(t *testing.T, m referenceframe.Model, q float64) map[string]spatialmath.Geometry {
	t.Helper()
	gif, err := m.Geometries([]referenceframe.Input{q})
	test.That(t, err, test.ShouldBeNil)
	out := map[string]spatialmath.Geometry{}
	for _, g := range gif.Geometries() {
		out[g.Label()] = g
	}
	return out
}

func TestGripperPulseDriveAngle(t *testing.T) {
	test.That(t, pulseToDriveAngle(850), test.ShouldAlmostEqual, 0)
	test.That(t, pulseToDriveAngle(0), test.ShouldAlmostEqual, gripperDriveMaxRad)
	// Out-of-range reads, like the -1 a closed G2 reports, clamp to the limits.
	test.That(t, pulseToDriveAngle(-1), test.ShouldAlmostEqual, gripperDriveMaxRad)
	test.That(t, pulseToDriveAngle(900), test.ShouldAlmostEqual, 0)

	for _, pulse := range []int{0, 1, 425, 849, 850} {
		test.That(t, driveAngleToPulse(pulseToDriveAngle(pulse)), test.ShouldEqual, pulse)
	}
	test.That(t, driveAngleToPulse(-0.1), test.ShouldEqual, 850)
	test.That(t, driveAngleToPulse(2), test.ShouldEqual, 0)
}

func TestGripperFingerModel(t *testing.T) {
	mf := loadFingerModel(t)
	test.That(t, mf.DoF()[0].Min, test.ShouldAlmostEqual, 0)
	test.That(t, mf.DoF()[0].Max, test.ShouldAlmostEqual, gripperDriveMaxRad, 1e-9)

	for _, q := range []float64{0, 0.4, gripperDriveMaxRad} {
		tcp, err := mf.Transform([]referenceframe.Input{q})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, spatialmath.PoseAlmostEqual(tcp, spatialmath.NewPoseFromPoint(r3.Vector{Z: 172})), test.ShouldBeTrue)

		geoms := geometriesByLabel(t, mf, q)
		test.That(t, geoms, test.ShouldHaveLength, 7)

		// The finger box is offset from the finger link by its fitted centre and,
		// the linkage being a parallelogram, never rotates.
		want := leftFingerOrigin(q).Add(r3.Vector{Y: -10.04, Z: 27.54})
		left := geoms[ModelNameGripper+":left_finger"]
		test.That(t, left.Pose().Point().Distance(want), test.ShouldBeLessThan, 0.01)
		test.That(t, spatialmath.OrientationAlmostEqual(left.Pose().Orientation(), spatialmath.NewZeroOrientation()), test.ShouldBeTrue)

		right := geoms[ModelNameGripper+":right_finger"]
		test.That(t, right.Pose().Point().Distance(r3.Vector{X: want.X, Y: -want.Y, Z: want.Z}), test.ShouldBeLessThan, 0.01)
	}

	// The pads close to a near-zero gap without crossing and open to the ~85 mm
	// stroke. A pad's inner face is its box centre less half the 31.93 mm depth.
	padGap := func(q float64) float64 {
		return 2 * (geometriesByLabel(t, mf, q)[ModelNameGripper+":left_finger"].Pose().Point().Y - 31.93/2)
	}
	test.That(t, padGap(gripperDriveMaxRad), test.ShouldBeBetween, 0, 3)
	test.That(t, padGap(0), test.ShouldBeBetween, 84, 92)
}

// Without use_urdfs the fingers move the same way, in the box model's frame:
// the case box behind the origin, the mount face at -case length, and the
// reported pose at the origin rather than the TCP.
func TestGripperBoxFingerModel(t *testing.T) {
	urdfModel := loadFingerModel(t)

	for _, submodel := range []string{submodelG1, submodelG2} {
		t.Run(submodel, func(t *testing.T) {
			mf, err := standardGripperBoxModel(ModelNameGripper, submodel)
			test.That(t, err, test.ShouldBeNil)
			caseSize := standardGripperCaseSize(submodel)
			mount := spatialmath.NewPoseFromPoint(r3.Vector{Z: -caseSize.Z})

			for _, q := range []float64{0, 0.4, gripperDriveMaxRad} {
				pose, err := mf.Transform([]referenceframe.Input{q})
				test.That(t, err, test.ShouldBeNil)
				test.That(t, spatialmath.PoseAlmostEqual(pose, spatialmath.NewZeroPose()), test.ShouldBeTrue)

				geoms := geometriesByLabel(t, mf, q)
				caseBox := geoms[ModelNameGripper+":"+standardGripperCaseLabel]
				wantCase, err := spatialmath.NewBox(
					spatialmath.NewPoseFromPoint(r3.Vector{Z: -caseSize.Z / 2}), caseSize, caseBox.Label())
				test.That(t, err, test.ShouldBeNil)
				test.That(t, spatialmath.GeometriesAlmostEqual(caseBox, wantCase), test.ShouldBeTrue)

				for label, g := range geometriesByLabel(t, urdfModel, q) {
					if label == ModelNameGripper+":"+gripperURDFBaseLink {
						continue
					}
					test.That(t, spatialmath.PoseAlmostEqual(geoms[label].Pose(), spatialmath.Compose(mount, g.Pose())), test.ShouldBeTrue)
				}
			}
		})
	}
}

// A URDF can't name its end effector, so both finger models go over
// GetKinematics as SVA JSON; the receiver must rebuild the same model.
func TestGripperFingerModelsSurviveGetKinematics(t *testing.T) {
	urdfModel := loadFingerModel(t)
	boxModel, err := standardGripperBoxModel(ModelNameGripper, submodelG2)
	test.That(t, err, test.ShouldBeNil)

	for name, mf := range map[string]referenceframe.Model{"urdf": urdfModel, "box": boxModel} {
		t.Run(name, func(t *testing.T) {
			got, err := referenceframe.KinematicModelFromProtobuf(ModelNameGripper, referenceframe.KinematicModelToProtobuf(mf))
			test.That(t, err, test.ShouldBeNil)
			test.That(t, got.DoF(), test.ShouldHaveLength, 1)

			const q = 0.5
			have := geometriesByLabel(t, got, q)
			want := geometriesByLabel(t, mf, q)
			test.That(t, have, test.ShouldHaveLength, len(want))
			for label, g := range want {
				test.That(t, spatialmath.PoseAlmostEqual(have[label].Pose(), g.Pose()), test.ShouldBeTrue)
			}
		})
	}
}

// moving_fingers is opt-in: without it the standard gripper keeps its zero-DoF
// model for either use_urdfs setting, so nothing reads or moves the jaws.
func TestStandardGripperKinematicsMovingFingersOptIn(t *testing.T) {
	t.Setenv("VIAM_MODULE_ROOT", filepath.Dir(armDir()))
	logger := logging.NewTestLogger(t)

	for _, conf := range []GripperConfig{
		{},
		{UseURDFs: true},
		{MovingFingers: true},
		{MovingFingers: true, UseURDFs: true},
	} {
		t.Run(fmt.Sprintf("moving_fingers=%v,use_urdfs=%v", conf.MovingFingers, conf.UseURDFs), func(t *testing.T) {
			mf, err := newStandardGripperKinematics(&conf, submodelG2, logger)
			test.That(t, err, test.ShouldBeNil)
			if conf.MovingFingers {
				test.That(t, mf.DoF(), test.ShouldHaveLength, 1)
				return
			}
			test.That(t, mf.DoF(), test.ShouldBeEmpty)

			g := &myGripper{mf: mf, useURDFs: conf.UseURDFs, detected: detectedGripper{submodel: submodelG2}}
			inputs, err := g.CurrentInputs(context.Background())
			test.That(t, err, test.ShouldBeNil)
			test.That(t, inputs, test.ShouldBeEmpty)
			test.That(t, g.GoToInputs(context.Background(), []referenceframe.Input{0.4}), test.ShouldBeNil)

			geoms, err := g.Geometries(context.Background(), nil)
			test.That(t, err, test.ShouldBeNil)
			if conf.UseURDFs {
				test.That(t, geoms, test.ShouldHaveLength, 1)
				test.That(t, geoms[0].Label(), test.ShouldEqual, ModelNameGripper+":xarm_gripper_base_link")
				return
			}
			want, err := standardGripperGeometries(submodelG2)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, geoms, test.ShouldHaveLength, len(want))
			for i := range want {
				test.That(t, spatialmath.GeometriesAlmostEqual(geoms[i], want[i]), test.ShouldBeTrue)
			}
		})
	}
}
