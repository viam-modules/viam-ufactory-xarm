package arm

import (
	"context"
	_ "embed"
	"encoding/json"
	"errors"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// ModelNameXArm6MountingPlate is the model name of the xArm6 mounting plate obstacle.
const ModelNameXArm6MountingPlate = "xarm6_mounting_plate"

// MountingPlateModel is the xArm6 mounting plate: a static obstacle whose frame
// origin is the plate's bottom-face center and whose end frame is the
// bolt-circle center on its top face, where the arm mounts.
var MountingPlateModel = family.WithModel(ModelNameXArm6MountingPlate)

//go:embed xarm6_mounting_plate.json
var xArm6MountingPlateJSON []byte

var errMountingPlateStatic = errors.New("mounting plate is a static obstacle")

func init() {
	resource.RegisterComponent(
		gripper.API,
		MountingPlateModel,
		resource.Registration[gripper.Gripper, *MountingPlateConfig]{
			Constructor: newMountingPlate,
		})
}

// MountingPlateConfig takes no attributes; the frame's parent and pose place the plate.
type MountingPlateConfig struct{}

// Validate accepts every config.
func (c *MountingPlateConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

type mountingPlate struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name resource.Name
	mf   referenceframe.Model
}

func newMountingPlate(
	ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger,
) (gripper.Gripper, error) {
	if _, err := resource.NativeConfig[*MountingPlateConfig](config); err != nil {
		return nil, err
	}

	cfg := &referenceframe.ModelConfigJSON{
		OriginalFile: &referenceframe.ModelFile{Bytes: xArm6MountingPlateJSON, Extension: kinematicsSourceJSON},
	}
	if err := json.Unmarshal(xArm6MountingPlateJSON, cfg); err != nil {
		return nil, err
	}
	mf, err := cfg.ParseConfig(ModelNameXArm6MountingPlate)
	if err != nil {
		return nil, err
	}

	return &mountingPlate{name: config.ResourceName(), mf: mf}, nil
}

func (p *mountingPlate) Name() resource.Name {
	return p.name
}

func (p *mountingPlate) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	return false, errMountingPlateStatic
}

func (p *mountingPlate) Open(ctx context.Context, extra map[string]any) error {
	return errMountingPlateStatic
}

func (p *mountingPlate) Stop(ctx context.Context, extra map[string]any) error {
	return nil
}

func (p *mountingPlate) IsMoving(ctx context.Context) (bool, error) {
	return false, nil
}

func (p *mountingPlate) IsHoldingSomething(ctx context.Context, extra map[string]any) (gripper.HoldingStatus, error) {
	return gripper.HoldingStatus{}, nil
}

func (p *mountingPlate) Geometries(ctx context.Context, _ map[string]any) ([]spatialmath.Geometry, error) {
	gif, err := p.mf.Geometries([]referenceframe.Input{})
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (p *mountingPlate) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return p.mf, nil
}

func (p *mountingPlate) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (p *mountingPlate) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

func (p *mountingPlate) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}

func (p *mountingPlate) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return nil, resource.ErrDoUnimplemented
}
