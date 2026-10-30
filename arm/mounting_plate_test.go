package arm

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func newTestMountingPlate(t *testing.T) *mountingPlate {
	t.Helper()
	conf := resource.Config{
		Name:                "plate",
		API:                 gripper.API,
		Model:               MountingPlateModel,
		ConvertedAttributes: &MountingPlateConfig{},
	}
	g, err := newMountingPlate(context.Background(), nil, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	return g.(*mountingPlate)
}

func TestMountingPlateIsRegistered(t *testing.T) {
	test.That(t, MountingPlateModel.String(), test.ShouldEqual, "viam:ufactory:xarm6_mounting_plate")
	_, ok := resource.LookupRegistration(gripper.API, MountingPlateModel)
	test.That(t, ok, test.ShouldBeTrue)
}

// The plate has no joints, so it builds into the planner's frame system as a static link.
func TestMountingPlateIsZeroDoF(t *testing.T) {
	p := newTestMountingPlate(t)
	mf, err := p.Kinematics(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, mf.DoF(), test.ShouldBeEmpty)
	inputs, err := p.CurrentInputs(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, inputs, test.ShouldBeEmpty)
}

// The arm mounts at the bolt-circle center on the plate's top face.
func TestMountingPlateEndFrameIsBoltCircleCenter(t *testing.T) {
	p := newTestMountingPlate(t)
	mf, err := p.Kinematics(context.Background())
	test.That(t, err, test.ShouldBeNil)
	assertBoltCircleEndFrame(t, mf)
}

// The plate sits on its origin: bottom face at z=0, top face at z=8.
func TestMountingPlateGeometryRestsOnOrigin(t *testing.T) {
	p := newTestMountingPlate(t)
	geoms, err := p.Geometries(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	assertPlateBox(t, geoms)
}

// The arm's frame parent reaches it over GetKinematics, so the wire model must
// keep both the box and the bolt-circle offset.
func TestMountingPlateSurvivesGetKinematics(t *testing.T) {
	p := newTestMountingPlate(t)
	mf, err := p.Kinematics(context.Background())
	test.That(t, err, test.ShouldBeNil)

	rebuilt, err := referenceframe.KinematicModelFromProtobuf("plate", referenceframe.KinematicModelToProtobuf(mf))
	test.That(t, err, test.ShouldBeNil)
	assertBoltCircleEndFrame(t, rebuilt)
	assertPlateBox(t, modelGeometries(t, rebuilt))
}

func TestMountingPlateCannotActuate(t *testing.T) {
	p := newTestMountingPlate(t)
	grabbed, err := p.Grab(context.Background(), nil)
	test.That(t, err, test.ShouldBeError, errMountingPlateStatic)
	test.That(t, grabbed, test.ShouldBeFalse)
	test.That(t, p.Open(context.Background(), nil), test.ShouldBeError, errMountingPlateStatic)
}

func assertBoltCircleEndFrame(t *testing.T, mf referenceframe.Model) {
	t.Helper()
	end, err := mf.Transform([]referenceframe.Input{})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, end.Point(), test.ShouldResemble, r3.Vector{X: 0, Y: 5, Z: 8})
	test.That(t, spatialmath.OrientationAlmostEqual(end.Orientation(), spatialmath.NewZeroOrientation()), test.ShouldBeTrue)
}

func assertPlateBox(t *testing.T, geoms []spatialmath.Geometry) {
	t.Helper()
	test.That(t, geoms, test.ShouldHaveLength, 1)
	test.That(t, geoms[0].Pose().Point(), test.ShouldResemble, r3.Vector{X: 0, Y: 0, Z: 4})
	dims := geoms[0].ToProtobuf().GetBox().GetDimsMm()
	test.That(t, []float64{dims.GetX(), dims.GetY(), dims.GetZ()}, test.ShouldResemble, []float64{185, 136, 8})
}
