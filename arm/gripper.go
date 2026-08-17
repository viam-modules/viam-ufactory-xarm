package arm

import (
	"context"
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
	rutils "go.viam.com/rdk/utils"
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

// Raw Fn700/Fn702 pulse thresholds. The register scale (0..850 over the full
// stroke) is the same on G1 and G2 — only the millimetre mapping differs, and we
// never expose millimetres — so these serve both. On G2 they are a fallback:
// holding is read from the status register instead.
const fullyClosedThreshold = 10
const fullyOpenThreshold = 830
const fullyOpenPosition = 840

// G2 defaults. Speed is in raw Fn303/FnC01 pulses: UFactory's documented G2
// default is 2000 (G1's is 1500, which we leave to the gripper's own firmware
// rather than writing it). Force is the 1-100 percentage the SDKs default to in
// set_gripper_g2_position.
const (
	defaultGripperSpeedG2 = 2000
	defaultGripperForceG2 = 50
)

// gripperStatusTimeout bounds the G2 move poll, matching the SDKs' 10s default.
const gripperStatusTimeout = 10 * time.Second

// GripperConfig config for gripper.
type GripperConfig struct {
	Arm            string
	VacuumLengthMM float64 `json:"vacuum_length_mm"`
	GripperSpeed   int     `json:"gripper_speed,omitempty"`
	// GripperVersion pins the standard gripper's hardware generation to "g1" or
	// "g2", bypassing detection completely. Empty (the default) detects it by
	// probing for force-control support.
	GripperVersion string `json:"gripper_version,omitempty"`
	// GripperForce is the G2 grasp force as a percentage, 1-100. Ignored on G1,
	// which has no force register. 0 means defaultGripperForceG2.
	GripperForce int `json:"gripper_force,omitempty"`
	// ConnectionType overrides vacuum wiring detection: "plugin" or "contact".
	// Empty means auto-detect from the arm model.
	ConnectionType string `json:"connection_type,omitempty"`
	// UseURDFs opts into URDF-derived mesh geometries. When false (the
	// default) the gripper reports the hand-authored bounding boxes that
	// shipped before mesh support landed.
	UseURDFs bool `json:"use_urdfs,omitempty"`
	// MeshDecimationRatio only applies when UseURDFs is true. Each gripper
	// URDF has exactly one mesh, so a scalar is enough. Pointer so a
	// missing field (nil) is distinguishable from an explicit value; the
	// call site substitutes an internal default when nil. Must be in (0, 1]
	// when set — 0 is a validation error (not a "use the default" sentinel).
	MeshDecimationRatio *float64 `json:"mesh_decimation_ratio,omitempty"`
}

// Validate validates the config.
func (cfg *GripperConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if cfg.GripperSpeed != 0 && (cfg.GripperSpeed < 1 || cfg.GripperSpeed > 5000) {
		return nil, nil, fmt.Errorf("gripper_speed must be between 1 and 5000, got %d", cfg.GripperSpeed)
	}
	switch cfg.GripperVersion {
	case "", submodelG1, submodelG2:
	default:
		return nil, nil, fmt.Errorf(`gripper_version must be %q or %q, got %q`, submodelG1, submodelG2, cfg.GripperVersion)
	}
	if cfg.GripperForce != 0 && (cfg.GripperForce < 1 || cfg.GripperForce > 100) {
		return nil, nil, fmt.Errorf("gripper_force must be between 1 and 100, got %d", cfg.GripperForce)
	}
	switch cfg.ConnectionType {
	case "", string(connectionPlugin), string(connectionContact):
	default:
		return nil, nil, fmt.Errorf(`connection_type must be "plugin" or "contact", got %q`, cfg.ConnectionType)
	}
	if cfg.MeshDecimationRatio != nil {
		r := *cfg.MeshDecimationRatio
		if r <= 0 || r > 1 {
			return nil, nil, fmt.Errorf("mesh_decimation_ratio must be in (0, 1] when set, got %f", r)
		}
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

	name     resource.Name
	mf       referenceframe.Model
	useURDFs bool

	arm      arm.Arm
	isMoving atomic.Bool

	detected detectedGripper

	logger logging.Logger
}

func newGripperLite(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	mf, err := newGripperKinematics(ModelNameGripperLite, newConf, logger, liteGripperGeometries)
	if err != nil {
		return nil, fmt.Errorf("gripper_lite kinematics: %w", err)
	}

	g := &myGripperLite{
		name:     config.ResourceName(),
		mf:       mf,
		useURDFs: newConf.UseURDFs,
		logger:   logger,
		isMoving: atomic.Bool{},
	}

	g.arm, err = arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	g.detected = probeGripper(ctx, g.arm, gripperKindBio, logger)

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

// IsHoldingSomething reads DIGITAL_OUT (0x0A15) via TGPIO_R16B — see the
// gripperLiteActionIsClosed case in liteGripperAction.
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
	return gripper.HoldingStatus{IsHoldingSomething: isHolding}, nil
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
	if g.useURDFs {
		gif, err := g.mf.Geometries(make([]referenceframe.Input, len(g.mf.DoF())))
		if err != nil {
			return nil, err
		}
		return gif.Geometries(), nil
	}
	return liteGripperGeometries()
}

func (g *myGripperLite) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.mf, nil
}

func (g *myGripperLite) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (g *myGripperLite) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

func (g *myGripperLite) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}

type myGripper struct {
	resource.AlwaysRebuild

	name     resource.Name
	mf       referenceframe.Model
	useURDFs bool

	arm arm.Arm

	goToPositionLock sync.Mutex
	isMoving         atomic.Bool

	detected detectedGripper
	// speed and force are the resolved G2 FnCxx parameters. Unused on G1.
	speed, force uint16

	logger logging.Logger
}

func newGripper(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	a, err := arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	x, err := rutils.AssertType[*xArm](a)
	if err != nil {
		return nil, fmt.Errorf("standard gripper: %w", err)
	}

	// A pinned gripper_version is taken at face value; otherwise probe. Either way
	// this has to settle before the kinematics model is built, because G1 and G2
	// report different collision geometry.
	detected := detectedGripper{kind: gripperKindStandard, submodel: newConf.GripperVersion}
	if newConf.GripperVersion != "" {
		logger.Infof("standard gripper: configured as %s, skipping detection", newConf.GripperVersion)
	} else if detected, err = resolveGripperSubmodel(ctx, x, logger); err != nil {
		return nil, err
	}
	submodel := detected.submodel

	if newConf.UseURDFs && submodel == submodelG2 {
		logger.Warn("gripper: use_urdfs is set but only the G1 mesh ships, so this G2 will report G1 collision geometry; " +
			"unset use_urdfs to get the G2 bounding boxes")
	}
	mf, err := newGripperKinematics(ModelNameGripper, newConf, logger, func() ([]spatialmath.Geometry, error) {
		return standardGripperGeometries(submodel)
	})
	if err != nil {
		return nil, fmt.Errorf("gripper kinematics: %w", err)
	}

	g := &myGripper{
		name:     config.ResourceName(),
		mf:       mf,
		useURDFs: newConf.UseURDFs,
		arm:      a,
		detected: detected,
		force:    defaultGripperForceG2,
		logger:   logger,
	}
	if newConf.GripperForce != 0 {
		g.force = uint16(newConf.GripperForce) //nolint:gosec // Validate bounds it to 1-100.
	}

	// G1 keeps its historical behaviour: only write Fn303 when the user asked for a
	// speed, otherwise leave the gripper on its own 1500 default. The G2 runs it
	// unconditionally for the setupGripper side effect — the gripper has to be
	// enabled before the first force-control write, and nothing else does that.
	g.speed = defaultGripperSpeedG2
	if newConf.GripperSpeed != 0 {
		g.speed = uint16(newConf.GripperSpeed)
	}
	if newConf.GripperSpeed != 0 || submodel == submodelG2 {
		if err := x.setupGripper(ctx); err != nil {
			return nil, fmt.Errorf("failed to set up gripper: %w", err)
		}
		if err := x.setGripperSpeed(ctx, g.speed); err != nil {
			return nil, fmt.Errorf("failed to set gripper speed: %w", err)
		}
	}

	return g, nil
}

// bus resolves the arm to the concrete type that owns the gripper Modbus
// transport.
func (g *myGripper) bus() (*xArm, error) {
	return rutils.AssertType[*xArm](g.arm)
}

// submodel is submodelG1 or submodelG2 and selects the grasp protocol: G1 uses
// the plain Fn700 position move plus a position-stall poll, G2 the force
// block-write plus the status register.
func (g *myGripper) submodel() string {
	return g.detected.submodel
}

func (g *myGripper) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	// G2 closes to 0, matching the SDK's clamp; G1 keeps its historical target of 2.
	if g.submodel() == submodelG2 {
		status, err := g.moveG2(ctx, 0)
		if err != nil {
			return false, err
		}
		return status&gripperStateMask == gripperStateDetected, nil
	}

	pos, err := g.goToPosition(ctx, 2)
	if err != nil {
		return false, err
	}

	return pos > fullyClosedThreshold, nil
}

func (g *myGripper) Open(ctx context.Context, extra map[string]any) error {
	if g.submodel() == submodelG2 {
		_, err := g.moveG2(ctx, fullyOpenPosition)
		return err
	}

	_, err := g.goToPosition(ctx, fullyOpenPosition)
	return err
}

// IsHoldingSomething reports the G2's own object-detected bit, which is
// authoritative the instant it is read. The G1 has no status register, so it
// keeps inferring holding from where the jaws came to rest.
func (g *myGripper) IsHoldingSomething(
	ctx context.Context,
	extra map[string]any,
) (gripper.HoldingStatus, error) {
	if g.submodel() == submodelG2 {
		status, err := g.getStatus(ctx)
		if err != nil {
			return gripper.HoldingStatus{}, err
		}
		meta := map[string]any{"status": status}
		// Position is best-effort here: it is useful telemetry but must not turn
		// a good holding answer into an error.
		if pos, err := g.getPosition(ctx); err == nil {
			meta["position"] = pos
		} else {
			g.logger.Debugf("gripper position read failed during IsHoldingSomething: %v", err)
		}
		return gripper.HoldingStatus{
			IsHoldingSomething: status&gripperStateMask == gripperStateDetected,
			Meta:               meta,
		}, nil
	}

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

// gripperForceControlRegs is the FnCxx block: enable, speed, force, position
// high, position low. Writing it applies force atomically with the move — the G1
// has no equivalent, which is also how the two are told apart.
const gripperForceControlRegs = 5

// writeForceControlBlock issues the G2 grasp write. Clearing FnC00 first makes the
// firmware see a 0->1 transition on the new write; without it, back-to-back
// grasps can be ignored while a hold is active.
//
// Package-level rather than a method so the arm's grab_with_torque DoCommand can
// share the one definition of this frame.
func writeForceControlBlock(ctx context.Context, x *xArm, speed, force uint16, position uint32) error {
	if err := x.disableGripperControlMode(ctx); err != nil {
		return err
	}
	return x.writeGripperRegisters(ctx, gripperControlModeReg, []uint16{
		1,
		speed,
		force,
		uint16(position >> 16),    //nolint:gosec // split of a 32-bit value.
		uint16(position & 0xFFFF), //nolint:gosec
	})
}

// moveG2 issues the force block-write and waits on the status register,
// returning the status that ended the move.
func (g *myGripper) moveG2(ctx context.Context, goal int) (uint16, error) {
	g.goToPositionLock.Lock()
	defer g.goToPositionLock.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	x, err := g.bus()
	if err != nil {
		return 0, err
	}
	if err := writeForceControlBlock(ctx, x, g.speed, g.force, uint32(goal)); err != nil { //nolint:gosec // goal is 0..850.
		return 0, err
	}

	return g.waitForStatus(ctx, gripperStatusTimeout)
}

// waitForStatus mirrors _check_gripper_status in both UFactory SDKs: a move is
// only complete once the controller has reported IS_MOTION and then settled back
// to IS_STOP or IS_DETECTED. Treating the first idle reading as "done" is what
// forced callers to sleep before IsHoldingSomething — immediately after the
// write the gripper has not begun moving, so its status and position still
// describe the previous pose. If motion never starts (already at the goal), the
// SDKs give up after 20 polls and call it done; we do the same.
func (g *myGripper) waitForStatus(ctx context.Context, timeout time.Duration) (uint16, error) {
	const pollInterval = 100 * time.Millisecond
	const notStartedPolls = 20

	started := false
	notStarted := 0
	deadline := time.Now().Add(timeout)
	var status uint16

	for time.Now().Before(deadline) {
		if !utils.SelectContextOrWait(ctx, pollInterval) {
			return status, ctx.Err()
		}
		var err error
		if status, err = g.getStatus(ctx); err != nil {
			return status, err
		}
		switch status & gripperStateMask {
		case gripperStateFault:
			return status, fmt.Errorf("gripper reported a fault (status 0x%04x)", status)
		case gripperStateMotion:
			started = true
		default: // gripperStateStop or gripperStateDetected
			if started {
				return status, nil
			}
			notStarted++
			if notStarted >= notStartedPolls {
				return status, nil
			}
		}
	}
	return status, fmt.Errorf("gripper move did not complete within %s (status 0x%04x)", timeout, status)
}

func (g *myGripper) getStatus(ctx context.Context) (uint16, error) {
	x, err := g.bus()
	if err != nil {
		return 0, err
	}
	r, err := x.readGripperRegisters(ctx, standardGripperStatusReg, 1)
	if err != nil {
		return 0, err
	}
	if r.exception != 0 {
		return 0, fmt.Errorf("gripper status read rejected with Modbus exception 0x%02X (%v)", r.exception, r.params)
	}
	w := r.words()
	if len(w) != 1 {
		return 0, fmt.Errorf("bad gripper status response %v", r.params)
	}
	return w[0], nil
}

func (g *myGripper) goToPosition(ctx context.Context, goal int) (int, error) {
	g.goToPositionLock.Lock()
	defer g.goToPositionLock.Unlock()

	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	x, err := g.bus()
	if err != nil {
		return 0, err
	}
	if err := x.setupGripper(ctx); err != nil {
		return 0, err
	}
	if err := x.setGripperPosition(ctx, uint32(goal)); err != nil { //nolint:gosec // goal is 0..850.
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
	x, err := g.bus()
	if err != nil {
		return 0, err
	}
	pos, err := x.getGripperPosition(ctx)
	return int(pos), err
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
	if g.useURDFs {
		gif, err := g.mf.Geometries(make([]referenceframe.Input, len(g.mf.DoF())))
		if err != nil {
			return nil, err
		}
		return gif.Geometries(), nil
	}
	return standardGripperGeometries(g.submodel())
}

func (g *myGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.mf, nil
}

func (g *myGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

func (g *myGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

func (g *myGripper) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}

func standardGripperGeometries(version string) ([]spatialmath.Geometry, error) {
	caseBoxSize := r3.Vector{X: 50, Y: 100, Z: 100}
	clawSize := r3.Vector{X: 40, Y: 170, Z: 105}
	if version == submodelG2 {
		caseBoxSize = r3.Vector{X: 75, Y: 110, Z: 110}
		clawSize = r3.Vector{X: 45, Y: 120, Z: 112}
	}

	caseBox, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: caseBoxSize.Z / -2}),
		caseBoxSize, "case-gripper")
	if err != nil {
		return nil, err
	}
	claws, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Z: 50 + (clawSize.Z / -2)}),
		clawSize, "claws")
	if err != nil {
		return nil, err
	}
	return []spatialmath.Geometry{caseBox, claws}, nil
}

// liteGripperGeometries — hand-authored boxes for the Lite gripper.
func liteGripperGeometries() ([]spatialmath.Geometry, error) {
	caseBoxSize := r3.Vector{X: 30, Y: 60, Z: 55.5}
	caseBox, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: caseBoxSize.Z / -2}),
		caseBoxSize, "case-gripper")
	if err != nil {
		return nil, err
	}
	clawSize := r3.Vector{X: 20, Y: 48, Z: 25}
	claws, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Z: caseBoxSize.Z/2 + (clawSize.Z / -2)}),
		clawSize, "claws")
	if err != nil {
		return nil, err
	}
	return []spatialmath.Geometry{caseBox, claws}, nil
}
