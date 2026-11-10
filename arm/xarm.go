// Package arm defines the arm that a robot uses to manipulate objects.
package arm

import (
	"context"
	"fmt"
	"net/http"
	"slices"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"
	"go.uber.org/multierr"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/mlmodel"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

const (
	maxSpeed = 180.  // degrees per second
	minSpeed = 0.25  // degrees per second
	maxAccel = 1145. // degrees per second per second

	defaultSpeed       = maxSpeed / 3 // degrees per second
	defaultAccel       = maxAccel / 3 // degrees per second per second
	defaultPort        = 502
	defaultGripperPort = 503
	defaultMoveHz      = 100. // Don't change this

	interwaypointAccel = 600. // degrees per second per second. All xarms max out at 1145

	defaultTrajGenPathToleranceDeltaRads             = 0.1
	defaultTrajGenWaypointDeduplicationToleranceRads = 1e-3

	defaultStreamLimitToleranceRatio = 1.01 // accept 1% tolerance on stream vel and acc limits

	// DoCommand keys.
	loadKey                  = "load"
	moveGripperKey           = "move_gripper"
	getGripperKey            = "get_gripper"
	gripperPositionKey       = "gripper_position"
	setAcckey                = "set_acceleration"
	setSpeedKey              = "set_speed"
	grabVacuumKey            = "grab_vacuum"
	openVacuumKey            = "open_vacuum"
	clearErrorKey            = "clear_error"
	getStateKey              = "get_state"
	getErrorKey              = "get_error"
	getVacuumGripperStateKey = "get_vacuum_state"
	vacuumGripperStateKey    = "vacuum_state"
	connectionTypeKey        = "connection_type"
	gripperLiteActionKey     = "gripper_lite_action"
	setGripperSpeedKey       = "set_gripper_speed"
	getGripperSpeedKey       = "get_gripper_speed"
	gripperSpeedKey          = "gripper_speed"
	grabWithTorqueKey        = "grab_with_torque"
	enterManualModeKey       = "enter_manual_mode"
	exitManualModeKey        = "exit_manual_mode"
	getFTSensorDataKey       = "get_ft_sensor_data"
	ftSensorZeroKey          = "ft_sensor_zero"
	ftSensorEnableKey        = "ft_sensor_enable"
	ftSensorDataKey          = "ft_sensor_data"

	// gripperLiteActionKeys.
	gripperLiteActionOpen     = "open"
	gripperLiteActionClose    = "close"
	gripperLiteActionIsClosed = "is_closed"
	gripperLiteActionStop     = "stop"

	kinematicsSourceJSON = "json"
	kinematicsBase       = "base"
)

const (
	// ModelName6DOF is the name of a UFactory xArm 6.
	ModelName6DOF = "xArm6"
	// ModelName7DOF is the name of a UFactory xArm 7.
	ModelName7DOF = "xArm7"
	// ModelNameLite is the name of a UFactory Lite 6.
	ModelNameLite = "lite6"
	// ModelName850 is the name of a UFactory 850.
	ModelName850 = "xArm850"
)

var (
	family = resource.ModelNamespace("viam").WithFamily("ufactory")

	// XArm6Model defines the resource.Model for the xArm6.
	XArm6Model = family.WithModel(ModelName6DOF)
	// XArm7Model defines the resource.Model for the xArm7.
	XArm7Model = family.WithModel(ModelName7DOF)
	// XArmLite6Model defines the resource.Model for the lite6.
	XArmLite6Model = family.WithModel(ModelNameLite)
	// XArm850Model defines the resource.Model for the 850.
	XArm850Model = family.WithModel(ModelName850)
)

type xArm struct {
	resource.AlwaysRebuild

	// cmdConn carries arm motion, state queries, and direct TGPIO writes
	// (port 502). gripperConn carries gripper Modbus passthrough traffic
	// (port 503 when reachable, otherwise aliased to cmdConn).
	cmdConn     *modbusConn
	gripperConn *modbusConn

	// stateWrite records when a command that can change the controller's mode or
	// state was last sent (see markStateWrite); nil until the first one. Report
	// packets received before it may not reflect that command yet. It is a
	// time.Time rather than Unix nanoseconds so comparisons with packet arrival
	// times use the monotonic clock and survive wall-clock jumps.
	stateWrite atomic.Pointer[time.Time]

	// manualExit auto-exits manual mode after SetManualMode's enabledFor elapses
	manualExit exitTimer

	name        resource.Name
	conf        *Config
	closed      atomic.Bool
	opMgr       *operation.SingleOperationManager
	logger      logging.Logger
	motion      motion.Service
	trajGen     mlmodel.Service
	proxyServer *http.Server
	// report streams the controller's rich report; nil until NewXArm starts it.
	report *reportStream

	// below is all configuration things
	dof    int
	model  referenceframe.Model
	moveHZ float64 // Number of joint positions to send per second

	// TODO: remove this lock and make this not settable
	confLock     sync.Mutex // speed and acceleration are both able to be read/written to, so they need to be protected by a mutex
	speed        float64    // speed=max joint radians per second
	acceleration float64    // acceleration= joint radians per second increase per second

	// gripperControlMode records whether the gripper's FnCxx block-write control mode may be
	// enabled. Only graspWithTorque turns it on, but it survives a process restart, so it starts
	// out true and the first gripper setup clears it.
	gripperControlMode atomic.Bool
	// gripperSpeed is the last speed written to the gripper's Fn303 register, or 0 if none was
	// ever set. Clearing the FnCxx control mode resets Fn303, so we need this to put it back.
	gripperSpeed atomic.Uint32

	// ftEnabled records whether F/T sampling was ever turned on. An F/T fault makes the
	// controller stop sampling and clearing the error does not restart it, so checkReadyState
	// replays the enable. Same shape as gripperControlMode/gripperSpeed above.
	ftEnabled atomic.Bool
	// ftRestoreNano rate-limits that replay; see ftRestoreCooldown.
	ftRestoreNano atomic.Int64

	// gripperPreset is the payload preset of the mounted UFACTORY gripper,
	// applied only when the controller reports no payload.
	gripperPreset atomic.Pointer[gripperPreset]
	// payloadChecked records whether ensureTCPPayload has run on the current
	// connection. A controller reboot drops our socket and reverts the payload
	// to its saved value, so this is cleared on every connection reset, and
	// whenever a gripper registers a preset.
	payloadChecked atomic.Bool

	detectedArm detectedArm
}

func init() {
	for _, model := range []resource.Model{XArm6Model, XArm7Model, XArmLite6Model, XArm850Model} {
		register(model)
	}
}

func register(model resource.Model) {
	resource.RegisterComponent(
		arm.API,
		model,
		resource.Registration[arm.Arm, *Config]{
			Constructor: func(
				ctx context.Context,
				deps resource.Dependencies,
				conf resource.Config,
				logger logging.Logger,
			) (arm.Arm, error) {
				return newxArm(ctx, conf, logger, model.Name, deps)
			},
		},
	)
}

// TrajGenConfig holds configuration for the trajectory generator ML model service.
type TrajGenConfig struct {
	Service                            string   `json:"service"`
	PathToleranceDeltaRads             *float64 `json:"path_tolerance_delta_rads,omitempty"`
	PathColinearizationRatio           *float64 `json:"path_colinearization_ratio,omitempty"`
	WaypointDeduplicationToleranceRads *float64 `json:"waypoint_deduplication_tolerance_rads,omitempty"`
}

// Config is used for converting config attributes.
type Config struct {
	Host         string  `json:"host"`
	Port         int     `json:"port,omitempty"`
	Speed        float64 `json:"speed_degs_per_sec,omitempty"`
	Acceleration float64 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
	MoveHZ       float64 `json:"move_hz,omitempty"`
	Sensitivity  *int    `json:"collision_sensitivity,omitempty"`
	BadJoints    []int   `json:"bad-joints"`
	Motion       string  `json:"motion"`
	// CollisionMeshes gives the links their collision meshes, which plan slower and travel
	// larger, in place of the boxes bounding them.
	CollisionMeshes           bool           `json:"collision_meshes,omitempty"`
	TrajGen                   *TrajGenConfig `json:"trajectory_generator,omitempty"`
	StreamLimitToleranceRatio float64        `json:"stream_limit_tolerance_ratio,omitempty"`

	StudioProxy     bool `json:"ufactory-studio-proxy,omitempty"`
	StudioProxyPort int  `json:"ufactory-studio-proxy-port,omitempty"`
}

// Validate validates the config.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	if cfg.Acceleration < 0 {
		return nil, nil, fmt.Errorf("given acceleration %f cannot be negative", cfg.Acceleration)
	}

	if cfg.Acceleration > maxAccel {
		return nil, nil, fmt.Errorf("given acceleration %f cannot be more than %f", cfg.Acceleration, maxAccel)
	}

	if cfg.Speed != 0 && (cfg.Speed < minSpeed || cfg.Speed > maxSpeed) {
		return nil, nil, fmt.Errorf("given speed %f must be between %f and %f", cfg.Speed, minSpeed, maxSpeed)
	}

	if cfg.MoveHZ > 0 && (cfg.MoveHZ < 20 || cfg.MoveHZ > 1000) {
		return nil, nil, fmt.Errorf("MoveHZ has to be between 20 and 1000")
	}

	if cfg.Sensitivity != nil && (*cfg.Sensitivity < 0 || *cfg.Sensitivity > 5) {
		return nil, nil, fmt.Errorf("given collision sensitivity %d is invalid, must be 0-5", cfg.Sensitivity)
	}

	if cfg.StreamLimitToleranceRatio != 0 && cfg.StreamLimitToleranceRatio < 1 {
		return nil, nil, fmt.Errorf("stream_limit_tolerance_ratio must be >= 1, got %f", cfg.StreamLimitToleranceRatio)
	}

	deps := []string{}
	opt := []string{}

	if cfg.Motion != "" {
		deps = append(deps, motion.Named(cfg.Motion).String())
	} else {
		opt = append(opt, motion.Named("builtin").String())
	}

	if cfg.TrajGen != nil && cfg.TrajGen.Service != "" {
		deps = append(deps, cfg.TrajGen.Service)
	}
	return deps, opt, nil
}

func (cfg *Config) speed() float32 {
	if cfg.Speed == 0 {
		return defaultSpeed
	}
	return float32(cfg.Speed)
}

func (cfg *Config) acceleration() float32 {
	if cfg.Acceleration == 0 {
		return defaultAccel
	}
	return float32(cfg.Acceleration)
}

func (cfg *Config) moveHZ() float64 {
	if cfg.MoveHZ <= 0 {
		return defaultMoveHz
	}
	return cfg.MoveHZ
}

func (cfg *Config) host() string {
	port := defaultPort
	if cfg.Port > 0 {
		port = cfg.Port
	}
	return fmt.Sprintf("%s:%d", cfg.Host, port)
}

func (cfg *Config) maxBadJoint() int {
	maxJoint := -1
	for _, j := range cfg.BadJoints {
		if j > maxJoint {
			maxJoint = j
		}
	}
	return maxJoint
}

func (cfg *Config) streamLimitToleranceRatio() float64 {
	if cfg.StreamLimitToleranceRatio == 0 {
		return defaultStreamLimitToleranceRatio
	}
	return cfg.StreamLimitToleranceRatio
}

// lockedJointRangeDegs is the window a locked joint is narrowed to, a degree either side of
// where it is now, so IK does not fail outright on a reported position that drifts a little,
// clamped to the joint's own range since a bound past the stop would widen the hardware range
// and the model refuses that. A joint reporting a position outside its range cannot be locked
// where it is, and ok is false, since a window pulled inside the range would have the planner
// place everything downstream of the joint somewhere the arm is not.
func lockedJointRangeDegs(current referenceframe.Input, joint referenceframe.JointConfig) (lo, hi float64, ok bool) {
	now := utils.RadToDeg(current)
	if now < joint.Min || now > joint.Max {
		return 0, 0, false
	}
	return max(now-1, joint.Min), min(now+1, joint.Max), true
}

// MakeModelFrame returns the kinematics model of the arm, named for the resource, with the
// configured limits and properties on it.
//
// When armTypeCode matches a known hardware variant, 1305 on the xArm6, the variant's model
// file is used. Pass 0 when variant information is not available. The links carry their
// collision meshes when mesh is set and the boxes bounding them otherwise.
//
// speedDegsPerSec and accelDegsPerSec2 become every joint's velocity and acceleration limits, so
// the motion service plans against the speeds the arm moves at. Pass 0 for either to leave the
// file's limits alone. A value above a limit the file holds is an error naming the joint, since
// a limit can only narrow. A joint in badJoints is locked to a degree either side of where it
// is now, and one reporting a position outside its range is an error, since the lock could not
// be put where the joint is. moveHz is published as the rate the arm expects joint positions at.
func MakeModelFrame(
	resourceName string,
	modelName string,
	badJoints []int,
	current []referenceframe.Input,
	mesh bool,
	logger logging.Logger,
	armTypeCode int,
	speedDegsPerSec float64,
	accelDegsPerSec2 float64,
	moveHz float64,
) (referenceframe.Model, error) {
	artifact, err := resolveArmKinematicsArtifact(modelName, detectedArm{armTypeCode: armTypeCode})
	if err != nil {
		return nil, err
	}
	model, err := loadKinematics(artifact.file, resourceName, mesh)
	if err != nil {
		return nil, err
	}
	cfg := model.ModelConfig()

	// The locks index by joint number, so an out of range entry would either panic or, worse,
	// be quietly skipped. A joint appears here because it is broken, so failing to lock one is
	// not something to discover later.
	for _, j := range badJoints {
		if j < 0 || j >= len(cfg.Joints) || j >= len(current) {
			return nil, fmt.Errorf("bad-joints index %d is out of range for %s, which has %d joints and %d reported positions",
				j, modelName, len(cfg.Joints), len(current))
		}
	}

	limits := make(map[string]referenceframe.JointLimits, len(cfg.Joints))
	for i, joint := range cfg.Joints {
		entry := referenceframe.JointLimits{}
		if speedDegsPerSec > 0 && accelDegsPerSec2 > 0 {
			entry.MaxVelocity, entry.MaxAcceleration = &speedDegsPerSec, &accelDegsPerSec2
		}
		if slices.Contains(badJoints, i) {
			lo, hi, ok := lockedJointRangeDegs(current[i], joint)
			if !ok {
				return nil, fmt.Errorf("joint %d (%s) reports %v degrees, outside its range [%v, %v], so it cannot be locked where it is",
					i, joint.ID, utils.RadToDeg(current[i]), joint.Min, joint.Max)
			}
			entry.Min, entry.Max = &lo, &hi
			logger.Infof("locking joint %d to %v", i, utils.RadToDeg(current[i]))
		}
		if entry != (referenceframe.JointLimits{}) {
			limits[joint.ID] = entry
		}
	}
	if err := model.NarrowJointLimits(limits); err != nil {
		return nil, err
	}
	model.SetKinematicProperties(referenceframe.KinematicProperties{OperatingFreqHz: moveHz})

	variant := artifact.variant
	if variant == "" {
		variant = kinematicsBase
	}
	logger.Infof("kinematics: model=%s variant=%s file=%s collision_meshes=%v", modelName, variant, artifact.file, mesh)
	return model, nil
}

// newxArm returns a new xArm of the specified modelName.
func newxArm(ctx context.Context, conf resource.Config, logger logging.Logger, modelName string, deps resource.Dependencies) (arm.Arm, error) {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}
	newConf.CollisionMeshes = wantsMesh(conf, newConf.CollisionMeshes, logger)

	return NewXArm(ctx, conf.ResourceName(), newConf, logger, modelName, deps)
}

// NewXArm creates a new x arm connection.
func NewXArm(ctx context.Context, name resource.Name,
	newConf *Config, logger logging.Logger, modelName string, deps resource.Dependencies) (arm.Arm, error) {
	var err error
	x := xArm{
		name:   name,
		conf:   newConf,
		moveHZ: newConf.moveHZ(),
		opMgr:  operation.NewSingleOperationManager(),
		logger: logger,

		acceleration: utils.DegToRad(float64(newConf.acceleration())),
		speed:        utils.DegToRad(float64(newConf.speed())),
	}
	x.cmdConn = newModbusConn(newConf.host(), logger, func() {
		x.payloadChecked.Store(false)
	})
	x.gripperConn = x.cmdConn // overwritten below if port 503 connects
	x.gripperControlMode.Store(true)

	if newConf.Motion != "" {
		if deps == nil {
			return nil, fmt.Errorf("no deps")
		}
		x.motion, err = motion.FromProvider(deps, newConf.Motion)
		if err != nil {
			return nil, err
		}
	} else {
		x.motion, err = motion.FromProvider(deps, "builtin")
		if err != nil {
			logger.Debugf("couldn't get default motion: %v", err)
		}
	}

	if newConf.TrajGen != nil && newConf.TrajGen.Service != "" {
		x.trajGen, err = mlmodel.FromProvider(deps, newConf.TrajGen.Service)
		if err != nil {
			return nil, err
		}
		if newConf.TrajGen.PathToleranceDeltaRads == nil {
			v := defaultTrajGenPathToleranceDeltaRads
			newConf.TrajGen.PathToleranceDeltaRads = &v
		}
		if newConf.TrajGen.PathColinearizationRatio == nil {
			v := 0.0
			newConf.TrajGen.PathColinearizationRatio = &v
		}
		if newConf.TrajGen.WaypointDeduplicationToleranceRads == nil {
			v := defaultTrajGenWaypointDeduplicationToleranceRads
			newConf.TrajGen.WaypointDeduplicationToleranceRads = &v
		}
	}

	err = x.connect(ctx)
	if err != nil {
		return nil, err
	}

	// Started right after the command socket so its first packet arrives while
	// hardware detection runs.
	x.report, err = newReportStream(ctx, reportAddr(newConf.Host), logger)
	if err != nil {
		return nil, multierr.Combine(err, x.Close(ctx))
	}

	// Try to open a dedicated socket on port 503 for gripper-bus Modbus
	// traffic. The xArm controller serves this as a sibling of port 502
	// specifically so gripper writes don't contend with arm motion on the
	// main socket. Falls back to the shared port-502 connection if 503 is
	// not reachable — behavior then matches a single-socket build.
	gripperAddr := fmt.Sprintf("%s:%d", newConf.Host, defaultGripperPort)
	gripperConn := newModbusConn(gripperAddr, logger, nil)
	if err := gripperConn.connect(ctx); err != nil {
		logger.Warnf("could not open port %d for gripper Modbus, falling back to shared port %d (gripper writes will contend with arm traffic): %v",
			defaultGripperPort, defaultPort, err)
	} else {
		x.gripperConn = gripperConn
		logger.Infof("gripper Modbus traffic routed through dedicated port %d", defaultGripperPort)
	}

	if d, err := x.detectArm(ctx); err != nil {
		logger.Warnf("xArm hardware detection failed: %v", err)
	} else {
		x.detectedArm = d
		if d.armTypeCode != 0 {
			logger.Infof(
				"xArm hardware detected: model=%s axis=%d device_type=%d submodel=%s arm_type=%d control_type=%d fw=%s (configured as %s)",
				d.model, d.axis, d.deviceType, d.submodel, d.armTypeCode, d.controlTypeCode, d.firmwareVersion, modelName,
			)
		} else {
			logger.Infof("xArm hardware detected: model=%s axis=%d device_type=%d (configured as %s)",
				d.model, d.axis, d.deviceType, modelName)
		}
	}

	// x.dof is not known yet, so start() could not check every joint's servo
	// and brake; set the arm up unconditionally instead.
	err = x.setupMotion(ctx, servoMotionMode)
	if err != nil {
		logger.Warnf("the xArm couldn't be started because: %s clear the error status before issuing command to the arm", err)
	}

	current := []referenceframe.Input{}
	if len(newConf.BadJoints) > 0 {
		x.dof = newConf.maxBadJoint() + 1
		current, err = x.CurrentInputs(ctx)
		if err != nil {
			return nil, multierr.Combine(err, x.Close(ctx))
		}
	}

	// The effective speeds, not the raw config ones, because an arm with no speed configured
	// still moves at the module default and the model should say so.
	x.model, err = MakeModelFrame(
		name.Name, modelName, newConf.BadJoints, current, newConf.CollisionMeshes, logger, x.detectedArm.armTypeCode,
		float64(newConf.speed()), float64(newConf.acceleration()), x.moveHZ,
	)
	if err != nil {
		return nil, multierr.Combine(err, x.Close(ctx))
	}
	x.dof = len(x.model.DoF())

	if len(current) > 0 {
		logger.Infof("model that was loaded config")
		for j, jc := range x.model.ModelConfig().Joints {
			logger.Infof("\t j: %d c: %v", j, jc)
		}
	}

	if newConf.Sensitivity != nil {
		err = x.setCollisionDetectionSensitivity(ctx, *newConf.Sensitivity)
		if err != nil {
			return nil, multierr.Combine(err, x.Close(ctx))
		}
	}

	if newConf.StudioProxy {
		if err := x.startProxy(ctx); err != nil {
			return nil, multierr.Combine(err, x.Close(ctx))
		}
	}

	return &x, nil
}

type moveOptions struct {
	speed        float64
	acceleration float64
	moveHZ       float64

	direct      bool
	waitAtEnd   bool
	interpolate bool
}

func f64(extra map[string]any, n string) (float64, bool) {
	v, ok := extra[n]
	if !ok {
		return 0, false
	}

	switch x := v.(type) {
	case float64:
		return x, true
	case int:
		return float64(x), true
	default:
		return 0, false
	}
}

func (x *xArm) moveOptions(opts *arm.MoveOptions, extra map[string]any) moveOptions {
	x.confLock.Lock()
	defer x.confLock.Unlock()

	o := moveOptions{
		speed:        x.speed,
		acceleration: x.acceleration,
		moveHZ:       x.moveHZ,
		direct:       false,
		waitAtEnd:    true,
		interpolate:  true,
	}

	if opts != nil {
		if opts.MaxVelRads != 0 {
			o.speed = opts.MaxVelRads
		}

		if opts.MaxAccRads != 0 {
			o.acceleration = opts.MaxAccRads
		}
	}

	if extra != nil {
		v, ok := f64(extra, "speed_r")
		if ok {
			o.speed = v
		}

		v, ok = f64(extra, "speed_d")
		if ok {
			o.speed = utils.DegToRad(v)
		}

		v, ok = f64(extra, "acceleration_r")
		if ok {
			o.acceleration = v
		}

		v, ok = f64(extra, "acceleration_d")
		if ok {
			o.acceleration = utils.DegToRad(v)
		}

		if extra["direct"] == true {
			o.direct = true
		}

		if extra["waitAtEnd"] == false {
			o.waitAtEnd = false
		}

		if extra["interpolate"] == false {
			o.interpolate = false
		}
	}

	o.speed = x.clampMoveOptions(
		o.speed,
		utils.DegToRad(minSpeed),
		utils.DegToRad(maxSpeed),
		"max velocity",
	)

	o.acceleration = x.clampMoveOptions(
		o.acceleration,
		0,
		utils.DegToRad(maxAccel),
		"max acceleration",
	)

	return o
}

func (x *xArm) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return x.JointPositions(ctx, nil)
}

// MoveToJointPositions moves the arm to the requested joint positions.
func (x *xArm) MoveToJointPositions(ctx context.Context, newPositions []referenceframe.Input, extra map[string]any) error {
	return x.MoveThroughJointPositions(ctx, [][]referenceframe.Input{newPositions}, nil, extra)
}

func (x *xArm) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return x.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (x *xArm) Geometries(ctx context.Context, extra map[string]any) ([]spatialmath.Geometry, error) {
	inputs, err := x.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := x.model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

// Get3DModels serves the GLB visuals the model carries, keyed by the link each one shows.
func (x *xArm) Get3DModels(ctx context.Context, extra map[string]any) (map[string]*commonpb.Mesh, error) {
	models := make(map[string]*commonpb.Mesh)
	sm, ok := x.model.(*referenceframe.SimpleModel)
	if !ok {
		return models, nil
	}
	for _, link := range sm.ModelConfig().Links {
		for _, visual := range sm.VisualGeometries(link.ID) {
			if visual.Type != spatialmath.MeshType || visual.MeshContentType != "glb" || len(visual.MeshData) == 0 {
				continue
			}
			// the viewer still asks for the MIME spelling rather than the extension
			models[link.ID] = &commonpb.Mesh{Mesh: visual.MeshData, ContentType: "model/gltf-binary"}
		}
	}
	return models, nil
}

// Kinematics returns the model built at configure time, including the speed limits from the
// config. set_speed and set_acceleration change how the arm moves without rebuilding this, so
// after one of those the published limits are the configured ones rather than the current ones.
// That is deliberate: kinematics describe how the arm is set up, and a DoCommand is not a
// reconfigure.
func (x *xArm) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return x.model, nil
}

// connectionTypeFromCmd resolves the vacuum connection type for a DoCommand:
// an explicit connection_type wins; otherwise fall back to the detected submodel.
func connectionTypeFromCmd(cmd map[string]any, detectedSubmodel string) connectionType {
	if v, ok := cmd[connectionTypeKey].(string); ok {
		switch connectionType(v) {
		case connectionContact:
			return connectionContact
		case connectionPlugin:
			return connectionPlugin
		}
	}
	if detectedSubmodel == submodelV2 {
		return connectionContact
	}
	return connectionPlugin
}

func (x *xArm) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	resp := map[string]any{}
	validCommand := false

	if val, ok := cmd[setGripperSpeedKey]; ok {
		if err := x.setupGripper(ctx); err != nil {
			return nil, err
		}
		speed, err := utils.AssertType[float64](val)
		if err != nil {
			return nil, err
		}
		if speed <= 0 || speed > 5000 {
			return nil, fmt.Errorf("gripper speed must be between 1 and 5000, got %v", val)
		}
		if err := x.setGripperSpeed(ctx, uint16(speed)); err != nil {
			return nil, err
		}
		resp[gripperSpeedKey] = speed
		validCommand = true
	}

	if _, ok := cmd[getGripperSpeedKey]; ok {
		if err := x.setupGripper(ctx); err != nil {
			return nil, err
		}
		speed, err := x.getGripperSpeed(ctx)
		if err != nil {
			return nil, err
		}
		resp[gripperSpeedKey] = float64(speed)
		validCommand = true
	}

	if val, ok := cmd[grabWithTorqueKey]; ok {
		params, ok := val.(map[string]any)
		if !ok {
			return nil, fmt.Errorf("%s must be a map with keys position, speed, torque; got %T", grabWithTorqueKey, val)
		}
		positionF, err := utils.AssertType[float64](params["position"])
		if err != nil {
			return nil, fmt.Errorf("grab_with_torque.position: %w", err)
		}
		if positionF < 0 || positionF > 850 {
			return nil, fmt.Errorf("grab_with_torque.position must be between 0 and 850, got %v", positionF)
		}
		speedF, err := utils.AssertType[float64](params["speed"])
		if err != nil {
			return nil, fmt.Errorf("grab_with_torque.speed: %w", err)
		}
		if speedF <= 0 || speedF > 5000 {
			return nil, fmt.Errorf("grab_with_torque.speed must be between 1 and 5000, got %v", speedF)
		}
		torqueF, err := utils.AssertType[float64](params["torque"])
		if err != nil {
			return nil, fmt.Errorf("grab_with_torque.torque: %w", err)
		}
		if torqueF < 0 || torqueF > 100 {
			return nil, fmt.Errorf("grab_with_torque.torque must be between 0%% and 100%%, got %v", torqueF)
		}
		stall := time.Second
		if raw, ok := params["stall_seconds"]; ok {
			stallF, err := utils.AssertType[float64](raw)
			if err != nil {
				return nil, fmt.Errorf("grab_with_torque.stall_seconds: %w", err)
			}
			if stallF <= 0 {
				return nil, fmt.Errorf("grab_with_torque.stall_seconds must be > 0, got %v", stallF)
			}
			stall = time.Duration(stallF * float64(time.Second))
		}
		if err := writeForceControlBlock(ctx, x, uint16(speedF), uint16(torqueF), uint32(positionF)); err != nil {
			return nil, err
		}
		if err := x.waitForGripper(ctx, int(positionF), stall); err != nil {
			return nil, err
		}
		validCommand = true
	}

	if val, ok := cmd[moveGripperKey]; ok {
		if err := x.setupGripper(ctx); err != nil {
			return nil, err
		}
		position, ok := val.(float64)
		if !ok || position < -10 || position > 850 {
			return nil, fmt.Errorf("must move gripper to an int between 0 and 840 %v", val)
		}
		if err := x.setGripperPosition(ctx, uint32(position)); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd[getGripperKey]; ok {
		pos, err := x.getGripperPosition(ctx)
		if err != nil {
			return nil, err
		}
		resp[gripperPositionKey] = float64(pos)
		validCommand = true
	}

	if _, ok := cmd[loadKey]; ok {
		loadInformation, err := x.getLoad(ctx)
		if err != nil {
			return nil, err
		}
		resp[loadKey] = loadInformation
		validCommand = true
	}
	if val, ok := cmd[setSpeedKey]; ok {
		speed, err := utils.AssertType[float64](val)
		if err != nil {
			return nil, err
		}
		if speed <= 0 {
			return nil, errors.New("speed cannot be less than or equal to zero")
		}
		x.confLock.Lock()
		x.speed = utils.DegToRad(speed)
		x.confLock.Unlock()
		validCommand = true
	}
	if val, ok := cmd[setAcckey]; ok {
		acceleration, err := utils.AssertType[float64](val)
		if err != nil {
			return nil, err
		}
		if acceleration <= 0 {
			return nil, errors.New("acceleration cannot be less than or equal to zero")
		}
		x.confLock.Lock()
		x.acceleration = utils.DegToRad(acceleration)
		x.confLock.Unlock()
		validCommand = true
	}
	if _, ok := cmd[grabVacuumKey]; ok {
		_, ok := cmd[grabVacuumKey].(bool)
		if !ok {
			return nil, errors.New("could not read grab_vacuum")
		}
		ct := connectionTypeFromCmd(cmd, vacuumGripperSubmodel(x.detectedArm))
		if err := x.grabVacuum(ctx, ct); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd[openVacuumKey]; ok {
		_, ok := cmd[openVacuumKey].(bool)
		if !ok {
			return nil, errors.New("could not read close_vacuum")
		}
		ct := connectionTypeFromCmd(cmd, vacuumGripperSubmodel(x.detectedArm))
		if err := x.openVacuum(ctx, ct); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd[clearErrorKey]; ok {
		if err := x.checkReadyState(ctx, false); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd[getStateKey]; ok {
		c := x.newCmd(regMap["GetState"])
		sData, err := x.send(ctx, c, true)
		if err != nil {
			return nil, err
		}
		return map[string]any{"state": sData.params}, nil
	}
	if _, ok := cmd[getErrorKey]; ok {
		c := x.newCmd(regMap["GetError"])
		sData, err := x.send(ctx, c, true)
		if err != nil {
			return nil, err
		}

		return map[string]any{"error info": sData.params}, nil
	}
	if _, ok := cmd[getVacuumGripperStateKey]; ok {
		ct := connectionTypeFromCmd(cmd, vacuumGripperSubmodel(x.detectedArm))
		res, err := x.getVacuumStatus(ctx, ct)
		if err != nil {
			return nil, err
		}
		resp[vacuumGripperStateKey] = res
		validCommand = true
	}

	if action, ok := cmd[gripperLiteActionKey]; ok {
		act, ok := action.(string)
		if !ok {
			return nil, fmt.Errorf("action %v (%T) is not a string", action, action)
		}
		res, err := x.liteGripperAction(ctx, act)
		if err != nil {
			return nil, err
		}
		resp[gripperLiteActionKey] = res
		validCommand = true
	}

	if _, ok := cmd[enterManualModeKey]; ok {
		if err := x.enterManualMode(ctx); err != nil {
			return nil, err
		}
		resp["status"] = "entered manual mode"
		validCommand = true
	}

	if _, ok := cmd[exitManualModeKey]; ok {
		if err := x.exitManualMode(ctx); err != nil {
			return nil, err
		}
		resp["status"] = "exited manual mode"
		validCommand = true
	}

	if _, ok := cmd[getFTSensorDataKey]; ok {
		vals, err := x.getFTSensorData(ctx)
		if err != nil {
			return nil, err
		}
		resp[ftSensorDataKey] = ftReadingsMap(vals)
		validCommand = true
	}
	if _, ok := cmd[ftSensorZeroKey]; ok {
		if err := x.setFTSensorZero(ctx); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if val, ok := cmd[ftSensorEnableKey]; ok {
		// Any non-bool value keeps the historical "presence means enable" behavior.
		enable := true
		if b, isBool := val.(bool); isBool {
			enable = b
		}
		if err := x.setFTSensorEnable(ctx, enable); err != nil {
			return nil, err
		}
		validCommand = true
	}

	if !validCommand {
		return nil, errors.New("command not found")
	}
	return resp, nil
}

func (x *xArm) Name() resource.Name {
	return x.name
}

func (x *xArm) Properties(ctx context.Context, extra map[string]any) (arm.Properties, error) {
	return arm.Properties{SupportManualMode: true, SupportCartesianCommands: true}, nil
}

func (x *xArm) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}
