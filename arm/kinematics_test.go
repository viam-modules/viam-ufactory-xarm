package arm

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestMakeGeometryModelRequiresGeometries(t *testing.T) {
	_, err := makeGeometryModel(ModelNameGripper, nil)
	test.That(t, err, test.ShouldNotBeNil)
}

// The model must carry an OriginalFile, otherwise KinematicModelToProtobuf drops
// it and the gripper reaches the frame system with no collision geometry.
func TestMakeGeometryModelSurvivesGetKinematics(t *testing.T) {
	box, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: -50}),
		r3.Vector{X: 50, Y: 100, Z: 100}, "case-gripper")
	test.That(t, err, test.ShouldBeNil)

	mf, err := makeGeometryModel(ModelNameGripper, []spatialmath.Geometry{box})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, mf.ModelConfig().OriginalFile, test.ShouldNotBeNil)
	test.That(t, mf.ModelConfig().OriginalFile.Extension, test.ShouldEqual, "json")

	resp := referenceframe.KinematicModelToProtobuf(mf)
	test.That(t, resp.GetKinematicsData(), test.ShouldNotBeEmpty)

	rebuilt, err := referenceframe.KinematicModelFromProtobuf(ModelNameGripper, resp)
	test.That(t, err, test.ShouldBeNil)
	rebuiltGeoms := modelGeometries(t, rebuilt)
	test.That(t, rebuiltGeoms, test.ShouldHaveLength, 1)
	test.That(t, spatialmath.GeometriesAlmostEqual(rebuiltGeoms[0], box), test.ShouldBeTrue)
}

// Each gripper's default (non-URDF) model must reach a planner with the same
// bounding boxes its Geometries method reports.
func TestGripperModelsCarryBoxesThroughGetKinematics(t *testing.T) {
	logger := logging.NewTestLogger(t)

	for _, tc := range []struct {
		name  string
		geoms func() ([]spatialmath.Geometry, error)
	}{
		{
			ModelNameGripper,
			func() ([]spatialmath.Geometry, error) { return standardGripperGeometries(submodelG2) },
		},
		{ModelNameGripperLite, liteGripperGeometries},
		{
			ModelNameVacuumGripper,
			func() ([]spatialmath.Geometry, error) { return vacuumGripperGeometries(VacuumGripperModel, 30) },
		},
		{
			ModelNameVacuumGripperLite,
			func() ([]spatialmath.Geometry, error) { return vacuumGripperGeometries(VacuumGripperModelLite, 0) },
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			want, err := tc.geoms()
			test.That(t, err, test.ShouldBeNil)
			test.That(t, want, test.ShouldNotBeEmpty)

			mf, err := newGripperKinematics(tc.name, &GripperConfig{}, logger, tc.geoms)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, mf.DoF(), test.ShouldBeEmpty)

			// Round-trip the way the module server and its caller do.
			rebuilt, err := referenceframe.KinematicModelFromProtobuf(tc.name, referenceframe.KinematicModelToProtobuf(mf))
			test.That(t, err, test.ShouldBeNil)
			test.That(t, rebuilt.DoF(), test.ShouldBeEmpty)

			got := modelGeometries(t, rebuilt)
			test.That(t, got, test.ShouldHaveLength, len(want))
			for i := range want {
				// A model namespaces its geometry labels, as "arm:base_top" is.
				test.That(t, got[i].Label(), test.ShouldEqual, tc.name+":"+want[i].Label())
				test.That(t, spatialmath.GeometriesAlmostEqual(got[i], want[i]), test.ShouldBeTrue)
			}

			// Nothing is lost crossing the wire.
			before := modelGeometries(t, mf)
			test.That(t, before, test.ShouldHaveLength, len(got))
			for i := range before {
				test.That(t, got[i].Label(), test.ShouldEqual, before[i].Label())
				test.That(t, spatialmath.GeometriesAlmostEqual(got[i], before[i]), test.ShouldBeTrue)
			}
		})
	}
}

func modelGeometries(t *testing.T, mf referenceframe.Model) []spatialmath.Geometry {
	t.Helper()
	gif, err := mf.Geometries(make([]referenceframe.Input, len(mf.DoF())))
	test.That(t, err, test.ShouldBeNil)
	return gif.Geometries()
}
