package arm

import (
	"context"
	"sync/atomic"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

// VacuumGripperModel model for the ufactory vacuum gripper.
var VacuumGripperModel = family.WithModel("vacuum_gripper")

var caseBoxSize = r3.Vector{X: 50, Y: 100, Z: 100}

// VacuumGripperConfig config for gripper.
type VacuumGripperConfig struct {
	Arm    string
	Vacuum bool
}

// Validate validates the config.
func (cfg *VacuumGripperConfig) Validate(path string) ([]string, error) {
	if cfg.Arm == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "board")
	}

	return []string{cfg.Arm}, nil
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		VacuumGripperModel,
		resource.Registration[gripper.Gripper, *VacuumGripperConfig]{
			Constructor: newVacuumGripper,
		})
}

func newVacuumGripper(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*VacuumGripperConfig](config)
	if err != nil {
		return nil, err
	}

	g := &myVacuumGripper{
		name:   config.ResourceName(),
		conf:   newConf,
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

	conf *VacuumGripperConfig

	arm arm.Arm

	isMoving atomic.Bool

	logger logging.Logger
}

func (g *myVacuumGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	{
		_, err := g.arm.DoCommand(ctx, map[string]interface{}{
			"grab_vacuum": true,
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
			"open_vacuum": true,
		})
		if err != nil {
			return err
		}
	}
	return nil
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

func (g *myVacuumGripper) ModelFrame() referenceframe.Model {
	return nil
}
