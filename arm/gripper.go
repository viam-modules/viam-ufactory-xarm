package arm

import (
	"context"
	"errors"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"
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
func (cfg *GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "arm")
	}
	return []string{cfg.Arm}, nil, nil
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
		name:   config.ResourceName(),
		mf:     referenceframe.NewSimpleModel("foo"),
		logger: logger,
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

	arm arm.Arm

	goToPositionLock sync.Mutex
	isMoving         atomic.Bool

	logger logging.Logger
}

func (g *myGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	pos, err := g.goToPosition(ctx, 2)
	if err != nil {
		return false, err
	}

	return pos > 10, nil
}

func (g *myGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	_, err := g.goToPosition(ctx, 840)
	return err
}

func (g *myGripper) goToPosition(ctx context.Context, goal int) (int, error) {
	g.goToPositionLock.Lock()
	defer g.goToPositionLock.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	_, err := g.arm.DoCommand(ctx, map[string]interface{}{
		"setup_gripper": true,
		"move_gripper":  goal,
	})
	if err != nil {
		return 0, err
	}

	old := -1
	start := time.Now()

	for {
		time.Sleep(10 * time.Millisecond)

		pos, err := g.getPosition(ctx)
		if err != nil {
			return 0, err
		}

		if math.Abs(float64(pos-goal)) <= 6 {
			return pos, nil
		}

		if old >= 0 && math.Abs(float64(pos-old)) <= 3 {
			return pos, nil
		}

		old = pos
		if time.Since(start) > (2 * time.Second) {
			return 0, fmt.Errorf("goToPosition %d timed out after: %v", goal, time.Since(start))
		}
	}
}

func (g *myGripper) getPosition(ctx context.Context) (int, error) {
	res, err := g.arm.DoCommand(ctx, map[string]interface{}{
		"get_gripper": true,
	})
	if err != nil {
		return 0, err
	}

	raw := res["gripper_position"]
	pos, ok := raw.(float64)
	if !ok {
		return 0, fmt.Errorf("bad gripper_position (%v) %T", raw, raw)
	}
	return int(pos), nil
}

func (g *myGripper) Name() resource.Name {
	return g.name
}

func (g *myGripper) Close(ctx context.Context) error {
	return g.Stop(ctx, nil)
}

func (g *myGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["get"] == true {
		pos, err := g.getPosition(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"pos": pos}, nil
	}
	return map[string]interface{}{}, nil
}

func (g *myGripper) IsMoving(context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

func (g *myGripper) Stop(context.Context, map[string]interface{}) error {
	// TODO(erh): fix me
	return nil
}

func (g *myGripper) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	caseBoxSize := r3.Vector{X: 50, Y: 100, Z: 100}
	caseBox, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: caseBoxSize.Z / -2}), caseBoxSize, "case-gripper")
	if err != nil {
		return nil, err
	}

	pos, err := g.getPosition(ctx)
	if err != nil {
		return nil, err
	}

	clawSize := r3.Vector{X: 40, Y: 170, Z: 105} // size open

	if pos < 20 || true { // gripper is closed
		clawSize.Y = 110
		clawSize.Z = 130
	}

	g.logger.Debugf("clawSize: %v", clawSize)

	claws, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: 50 + (clawSize.Z / -2)}), clawSize, "claws")
	if err != nil {
		return nil, err
	}

	return []spatialmath.Geometry{
		caseBox,
		claws,
	}, nil
}

func (g *myGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.mf, nil
}

func (g *myGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *myGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}
