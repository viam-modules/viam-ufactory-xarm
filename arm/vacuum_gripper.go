package arm

import (
	"context"
	"errors"
	"fmt"
	"sync/atomic"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// ModelNameVacuumGripper is the ufactory vacuum gripper commonly attached to xArm6/xArm7/xArm850.
const ModelNameVacuumGripper = "vacuum_gripper"

// ModelNameVacuumGripperLite is the ufactory vacuum gripper commonly attached to the lite6.
const ModelNameVacuumGripperLite = "vacuum_gripper_lite"

// VacuumGripperModel model for the ufactory vacuum gripper.
var VacuumGripperModel = family.WithModel(ModelNameVacuumGripper)

// VacuumGripperModelLite is the ufactory vacuum gripper commonly attached to the lite6.
var VacuumGripperModelLite = family.WithModel(ModelNameVacuumGripperLite)

func init() {
	resource.RegisterComponent(
		gripper.API,
		VacuumGripperModel,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newVacuumGripper,
		})
	resource.RegisterComponent(
		gripper.API,
		VacuumGripperModelLite,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newVacuumGripper,
		})
}

func newVacuumGripper(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	mf, err := loadGripperModel(config.Model.Name)
	if err != nil {
		return nil, fmt.Errorf("%s kinematics: %w", config.Model.Name, err)
	}

	g := &myVacuumGripper{
		name:   config.ResourceName(),
		conf:   newConf,
		logger: logger,
		model:  config.Model,
		mf:     mf,
	}

	g.arm, err = arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	g.detected = probeGripper(ctx, g.arm, gripperKindVacuum, logger)

	return g, nil
}

type myVacuumGripper struct {
	resource.AlwaysRebuild

	name  resource.Name
	conf  *GripperConfig
	model resource.Model
	mf    referenceframe.Model

	arm arm.Arm

	isMoving atomic.Bool

	detected detectedGripper

	logger logging.Logger
}

func (g *myVacuumGripper) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	{
		_, err := g.arm.DoCommand(ctx, map[string]any{
			grabVacuumKey: true,
		})
		if err != nil {
			return false, err
		}
	}

	return true, nil
}

func (g *myVacuumGripper) Open(ctx context.Context, extra map[string]any) error {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	{
		_, err := g.arm.DoCommand(ctx, map[string]any{
			openVacuumKey: true,
		})
		if err != nil {
			return err
		}
	}
	return nil
}

func (g *myVacuumGripper) IsHoldingSomething(
	ctx context.Context,
	extra map[string]any,
) (gripper.HoldingStatus, error) {
	res, err := g.arm.DoCommand(ctx, map[string]any{
		getVacuumGripperStateKey: true,
	})
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	return gripper.HoldingStatus{IsHoldingSomething: res[vacuumGripperStateKey].(bool)}, nil
}

func (g *myVacuumGripper) Name() resource.Name {
	return g.name
}

func (g *myVacuumGripper) Close(ctx context.Context) error {
	return g.Stop(ctx, nil)
}

func (g *myVacuumGripper) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return map[string]any{}, nil
}

func (g *myVacuumGripper) IsMoving(context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

func (g *myVacuumGripper) Stop(context.Context, map[string]any) error {
	// TODO(erh): fix me
	return nil
}

func (g *myVacuumGripper) Geometries(ctx context.Context, _ map[string]any) ([]spatialmath.Geometry, error) {
	gif, err := g.mf.Geometries(make([]referenceframe.Input, len(g.mf.DoF())))
	if err != nil {
		return nil, err
	}
	geoms := gif.Geometries()

	// The vacuum suction tube length is configurable per deployment; reflect
	// it as an additional thin collision body extending past the model's TCP.
	if g.conf.VacuumLengthMM > 0 {
		tube, err := spatialmath.NewBox(
			spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: g.conf.VacuumLengthMM / 2}),
			r3.Vector{X: 5, Y: 5, Z: g.conf.VacuumLengthMM},
			"vacuum-gripper-tube")
		if err != nil {
			return nil, err
		}
		geoms = append(geoms, tube)
	}

	return geoms, nil
}

func (g *myVacuumGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.mf, nil
}

func (g *myVacuumGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *myVacuumGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

func (g *myVacuumGripper) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}
