package arm

import (
	"context"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

// GripperModel model for the ufactory gripper.
var GripperModel = family.WithModel("gripper")

// GripperConfig config for gripper.
type GripperConfig struct {
	Arm string
}

// Validate validates the config.
func (cfg *GripperConfig) Validate(path string) ([]string, error) {
	if cfg.Arm == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "board")
	}

	return []string{cfg.Arm}, nil
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		GripperModel,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newGripper,
		})
}

func newGripper(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	g := &myGripper{
		name: config.ResourceName(),
		mf:   referenceframe.NewSimpleModel("foo"),
		conf: newConf,
	}

	g.arm, err = arm.FromDependencies(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	return g, nil
}

type myGripper struct {
	resource.AlwaysRebuild

	name resource.Name
	mf   referenceframe.Model

	conf *GripperConfig

	arm arm.Arm
}

func (g *myGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	// TODO(erh): fix me
	return true, g.goToPosition(ctx, 0)
}

func (g *myGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	return g.goToPosition(ctx, 850)
}

func (g *myGripper) goToPosition(ctx context.Context, position int) error {
	_, err := g.arm.DoCommand(ctx, map[string]interface{}{
		"setup_gripper": true,
		"move_gripper":  position,
	})
	return err
}

func (g *myGripper) Name() resource.Name {
	return g.name
}

func (g *myGripper) Close(ctx context.Context) error {
	return nil
}

func (g *myGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (g *myGripper) IsMoving(context.Context) (bool, error) {
	// TODO(erh): fix me
	return false, nil
}

func (g *myGripper) Stop(context.Context, map[string]interface{}) error {
	// TODO(erh): fix me
	return nil
}

func (g *myGripper) Geometries(context.Context, map[string]interface{}) ([]spatialmath.Geometry, error) {
	// TODO(erh): fix me
	return []spatialmath.Geometry{}, nil
}

func (g *myGripper) ModelFrame() referenceframe.Model {
	return g.mf
}
