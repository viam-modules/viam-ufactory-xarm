package arm

import (
	"context"
	"errors"
	"sync/atomic"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// VacuumGripperModel model for the ufactory vacuum gripper.
var VacuumGripperModel = family.WithModel("vacuum_gripper")

var caseBoxSize = r3.Vector{X: 50, Y: 100, Z: 100}

func init() {
	resource.RegisterComponent(
		gripper.API,
		VacuumGripperModel,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newVacuumGripper,
		})
}

func newVacuumGripper(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	g := &myVacuumGripper{
		name:   config.ResourceName(),
		logger: logger,
	}

	g.arm, err = arm.FromDependencies(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	return g, nil
}

type myVacuumGripper struct {
	resource.AlwaysRebuild

	name resource.Name

	arm arm.Arm

	isMoving atomic.Bool

	logger logging.Logger
}

func (g *myVacuumGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	{
		_, err := g.arm.DoCommand(ctx, map[string]interface{}{
			grabVacuumKey: true,
		})
		if err != nil {
			return false, err
		}
	}

	return true, nil
}

func (g *myVacuumGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	{
		_, err := g.arm.DoCommand(ctx, map[string]interface{}{
			openVacuumKey: true,
		})
		if err != nil {
			return err
		}
	}
	return nil
}

// TODO (RSDK-11207): Actually implement this method.
func (g *myVacuumGripper) IsHoldingSomething(
	ctx context.Context,
	extra map[string]interface{},
) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, errors.ErrUnsupported
}

func (g *myVacuumGripper) Name() resource.Name {
	return g.name
}

func (g *myVacuumGripper) Close(ctx context.Context) error {
	return g.Stop(ctx, nil)
}

func (g *myVacuumGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (g *myVacuumGripper) IsMoving(context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

func (g *myVacuumGripper) Stop(context.Context, map[string]interface{}) error {
	// TODO(erh): fix me
	return nil
}

func (g *myVacuumGripper) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	caseBox, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: caseBoxSize.Z / -2}), caseBoxSize, "case-gripper")
	if err != nil {
		return nil, err
	}

	return []spatialmath.Geometry{
		caseBox,
	}, nil
}

func (g *myVacuumGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, errors.ErrUnsupported
}

func (g *myVacuumGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *myVacuumGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}
