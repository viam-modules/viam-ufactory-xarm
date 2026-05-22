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

const (
	// ModelNameGripper is the gripper commonly attached to xArm6/xArm7.
	ModelNameGripper = "gripper"
	// ModelNameGripperLite is the gripper commonly attached to the lite6.
	ModelNameGripperLite = "gripper_lite"
)

var (
	// GripperModel model for the ufactory gripper.
	GripperModel = family.WithModel(ModelNameGripper)
	// GripperModelLite model for the ufactory gripper-lite.
	GripperModelLite = family.WithModel(ModelNameGripperLite)
)

const fullyClosedThreshold = 10
const fullyOpenThreshold = 830

// GripperConfig config for gripper.
type GripperConfig struct {
	Arm            string
	VacuumLengthMM float64 `json:"vacuum_length_mm"`
	GripperSpeed   int     `json:"gripper_speed,omitempty"`
}

// Validate validates the config.
func (cfg *GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if cfg.GripperSpeed != 0 && (cfg.GripperSpeed < 1 || cfg.GripperSpeed > 5000) {
		return nil, nil, fmt.Errorf("gripper_speed must be between 1 and 5000, got %d", cfg.GripperSpeed)
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
	resource.RegisterComponent(
		gripper.API,
		GripperModelLite,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newGripperLite,
		})
}

type myGripperLite struct {
	resource.AlwaysRebuild

	name resource.Name

	arm      arm.Arm
	isMoving atomic.Bool

	logger logging.Logger
}

func newGripperLite(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	g := &myGripperLite{
		name:     config.ResourceName(),
		logger:   logger,
		isMoving: atomic.Bool{},
	}

	g.arm, err = arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	return g, nil
}

func (g *myGripperLite) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)
	if _, err := g.arm.DoCommand(ctx, map[string]any{
		gripperLiteActionKey: gripperLiteActionClose,
	}); err != nil {
		return false, err
	}
	return true, nil
}

func (g *myGripperLite) Open(ctx context.Context, extra map[string]any) error {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)
	_, err := g.arm.DoCommand(ctx, map[string]any{
		gripperLiteActionKey: gripperLiteActionOpen,
	})
	return err
}

func (g *myGripperLite) IsHoldingSomething(
	ctx context.Context,
	extra map[string]any,
) (gripper.HoldingStatus, error) {
	res, err := g.arm.DoCommand(ctx, map[string]any{
		gripperLiteActionKey: gripperLiteActionIsClosed,
	})
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	val, ok := res[gripperLiteActionKey]
	if !ok {
		return gripper.HoldingStatus{}, fmt.Errorf("command %s didn't return key %s instead got %+v", gripperLiteActionIsClosed, gripperLiteActionKey, res)
	}
	converted, ok := val.(map[string]any)
	if !ok {
		return gripper.HoldingStatus{}, fmt.Errorf("expected map[string]interface{} got %v of type %T", val, val)
	}
	isHoldingRaw, ok := converted[gripperLiteActionIsClosed]
	if !ok {
		return gripper.HoldingStatus{}, fmt.Errorf("response doesn't contain the key: %s have : %v", gripperLiteActionIsClosed, val)
	}
	isHolding, ok := isHoldingRaw.(bool)
	if !ok {
		return gripper.HoldingStatus{}, fmt.Errorf("key `%s` value is not a bool, %v is a %T", gripperLiteActionIsClosed, isHoldingRaw, isHoldingRaw)
	}

	return gripper.HoldingStatus{
		IsHoldingSomething: isHolding,
	}, nil
}

func (g *myGripperLite) Name() resource.Name {
	return g.name
}

func (g *myGripperLite) Close(ctx context.Context) error {
	return g.Stop(ctx, nil)
}

func (g *myGripperLite) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return map[string]any{}, nil
}

func (g *myGripperLite) IsMoving(context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

func (g *myGripperLite) Stop(ctx context.Context, extra map[string]any) error {
	defer g.isMoving.Store(false)
	_, err := g.arm.DoCommand(ctx, map[string]any{
		gripperLiteActionKey: gripperLiteActionStop,
	})
	return err
}

func (g *myGripperLite) Geometries(ctx context.Context, _ map[string]any) ([]spatialmath.Geometry, error) {
	caseBoxSize := r3.Vector{X: 30, Y: 60, Z: 55.5}
	caseBox, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: caseBoxSize.Z / -2}), caseBoxSize, "case-gripper")
	if err != nil {
		return nil, err
	}

	clawSize := r3.Vector{X: 20, Y: 48, Z: 25} // size open

	claws, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{Z: caseBoxSize.Z/2 + (clawSize.Z / -2)}), clawSize, "claws")
	if err != nil {
		return nil, err
	}

	return []spatialmath.Geometry{
		caseBox,
		claws,
	}, nil
}

func (g *myGripperLite) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, errors.ErrUnsupported
}

func (g *myGripperLite) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *myGripperLite) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

func (g *myGripperLite) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
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

func newGripper(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	g := &myGripper{
		name:   config.ResourceName(),
		mf:     referenceframe.NewSimpleModel("xarm-gripper"),
		logger: logger,
	}

	g.arm, err = arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	if newConf.GripperSpeed != 0 {
		if _, err := g.arm.DoCommand(ctx, map[string]any{
			setGripperSpeedKey: float64(newConf.GripperSpeed),
		}); err != nil {
			return nil, fmt.Errorf("failed to set gripper speed: %w", err)
		}
	}

	return g, nil
}

func (g *myGripper) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	pos, err := g.goToPosition(ctx, 2)
	if err != nil {
		return false, err
	}

	return pos > fullyClosedThreshold, nil
}

func (g *myGripper) Open(ctx context.Context, extra map[string]any) error {
	_, err := g.goToPosition(ctx, 840)
	return err
}

func (g *myGripper) IsHoldingSomething(
	ctx context.Context,
	extra map[string]any,
) (gripper.HoldingStatus, error) {
	pos, err := g.getPosition(ctx)
	if err != nil {
		return gripper.HoldingStatus{}, err
	}

	isHoldingSomething := pos > fullyClosedThreshold && pos < fullyOpenThreshold

	return gripper.HoldingStatus{
		IsHoldingSomething: isHoldingSomething,
		Meta: map[string]any{
			"position": pos,
		},
	}, nil
}

func (g *myGripper) goToPosition(ctx context.Context, goal int) (int, error) {
	g.goToPositionLock.Lock()
	defer g.goToPositionLock.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	_, err := g.arm.DoCommand(ctx, map[string]any{
		"setup_gripper": true,
		"move_gripper":  float64(goal),
	})
	if err != nil {
		return 0, err
	}

	old := -1
	start := time.Now()

	msSinceStuck := -1
	pollInterval := 30

	for {
		time.Sleep(time.Duration(pollInterval) * time.Millisecond)

		pos, err := g.getPosition(ctx)
		if err != nil {
			return 0, err
		}

		if math.Abs(float64(pos-goal)) <= 6 {
			return pos, nil
		}

		// if the gripper has stopped moving, return
		// might be grabbing something
		if old >= 0 && math.Abs(float64(pos-old)) <= 1 {
			msSinceStuck += pollInterval
			if msSinceStuck > 1000 {
				return pos, nil
			}
		} else {
			msSinceStuck = 0
		}

		old = pos
		// up timeout for high resistance grabs that take longer
		if time.Since(start) > (10 * time.Second) {
			return pos, nil
		}
	}
}

func (g *myGripper) getPosition(ctx context.Context) (int, error) {
	res, err := g.arm.DoCommand(ctx, map[string]any{
		getGripperKey: true,
	})
	if err != nil {
		return 0, err
	}

	raw := res[gripperPositionKey]
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

func (g *myGripper) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	if cmd["get"] == true {
		pos, err := g.getPosition(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]any{"pos": pos}, nil
	}
	if posF, ok := cmd["set"].(float64); ok {
		pos := int(posF)
		_, err := g.goToPosition(ctx, pos)
		if err != nil {
			return nil, err
		}
		pos, err = g.getPosition(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": pos}, nil
	}
	if _, ok := cmd[setGripperSpeedKey]; ok {
		return g.arm.DoCommand(ctx, cmd)
	}
	if _, ok := cmd[getGripperSpeedKey]; ok {
		return g.arm.DoCommand(ctx, cmd)
	}
	if _, ok := cmd[grabWithTorqueKey]; ok {
		g.isMoving.Store(true)
		defer g.isMoving.Store(false)
		return g.arm.DoCommand(ctx, cmd)
	}
	return map[string]any{}, nil
}

func (g *myGripper) IsMoving(context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

func (g *myGripper) Stop(context.Context, map[string]any) error {
	// TODO(erh): fix me
	return nil
}

func (g *myGripper) Geometries(ctx context.Context, _ map[string]any) ([]spatialmath.Geometry, error) {
	caseBoxSize := r3.Vector{X: 50, Y: 100, Z: 100}
	caseBox, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: caseBoxSize.Z / -2}), caseBoxSize, "case-gripper")
	if err != nil {
		return nil, err
	}

	clawSize := r3.Vector{X: 40, Y: 170, Z: 105} // size open

	if false {
		// until geometries aren't cacheed or model frame works differently can't do this
		pos, err := g.getPosition(ctx)
		if err != nil {
			return nil, err
		}

		if pos < 20 { // gripper is closed
			clawSize.Y = 110
			clawSize.Z = 130
		}
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
	return g.mf, fmt.Errorf("temp hack because of issues")
}

func (g *myGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *myGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

func (g *myGripper) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}
