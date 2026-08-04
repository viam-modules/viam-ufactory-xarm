package arm

import (
	"context"
	"testing"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestGripperKinematics(t *testing.T) {
	_, err := gripperKinematics(nil)
	test.That(t, err, test.ShouldNotBeNil)

	loaded, err := referenceframe.UnmarshalModelJSON(xArm6modeljson, ModelName6DOF)
	test.That(t, err, test.ShouldBeNil)
	got, err := gripperKinematics(loaded)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldEqual, loaded)
}

// An empty SimpleModel has no ModelConfig, so GetKinematics ships it with no
// kinematics data and the client rebuilds an equally empty model rather than
// falling back to Geometries(). Guards the reason Kinematics must error when it
// has no model to report.
func TestEmptyModelDoesNotSurviveGetKinematics(t *testing.T) {
	resp := referenceframe.KinematicModelToProtobuf(referenceframe.NewSimpleModel(ModelNameGripper))
	test.That(t, resp.GetKinematicsData(), test.ShouldBeEmpty)
	test.That(t, resp.GetFormat(), test.ShouldEqual, commonpb.KinematicsFileFormat_KINEMATICS_FILE_FORMAT_UNSPECIFIED)

	rebuilt, err := referenceframe.KinematicModelFromProtobuf(ModelNameGripper, resp)
	test.That(t, err, test.ShouldBeNil)
	gif, err := rebuilt.Geometries(nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, gif.Geometries(), test.ShouldBeEmpty)
}

// Without use_urdfs every gripper must report no kinematics and non-empty
// geometries, which is what drives RDK's client onto the Geometries() fallback
// that puts the hand-authored boxes into the frame system.
func TestGrippersReportBoxesWhenURDFsDisabled(t *testing.T) {
	ctx := context.Background()

	type kinematicGripper interface {
		Kinematics(ctx context.Context) (referenceframe.Model, error)
		Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error)
	}

	for _, tc := range []struct {
		name   string
		g      kinematicGripper
		labels []string
	}{
		{"gripper", &myGripper{}, []string{"case-gripper", "claws"}},
		{"gripper_lite", &myGripperLite{}, []string{"case-gripper", "claws"}},
		{
			"vacuum_gripper",
			&myVacuumGripper{model: VacuumGripperModel, conf: &GripperConfig{}},
			[]string{"vacuum-gripper-box", "vacuum-gripper-tube"},
		},
		{
			"vacuum_gripper_lite",
			&myVacuumGripper{model: VacuumGripperModelLite, conf: &GripperConfig{}},
			[]string{"vacuum-gripper-box", "vacuum-gripper-tube"},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := tc.g.Kinematics(ctx)
			test.That(t, err, test.ShouldNotBeNil)

			geoms, err := tc.g.Geometries(ctx, nil)
			test.That(t, err, test.ShouldBeNil)
			labels := make([]string, len(geoms))
			for i, geom := range geoms {
				labels[i] = geom.Label()
			}
			test.That(t, labels, test.ShouldResemble, tc.labels)
		})
	}
}
