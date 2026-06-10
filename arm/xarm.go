// Package arm defines the arm that a robot uses to manipulate objects.
package arm

import (
	"context"
	_ "embed" // for embedding model file.
	"encoding/json"
	"fmt"
	"net/http"
	"os"
	"sync"
	"sync/atomic"

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
	minSpeed = 3.    // degrees per second
	maxAccel = 1145. // degrees per second per second

	defaultSpeed       = maxSpeed / 3 // degrees per second
	defaultAccel       = maxAccel / 3 // degrees per second per second
	defaultPort        = 502
	defaultGripperPort = 503
	defaultMoveHz      = 100. // Don't change this

	interwaypointAccel = 600. // degrees per second per second. All xarms max out at 1145

	defaultTrajGenPathToleranceDeltaRads             = 0.1
	defaultTrajGenWaypointDeduplicationToleranceRads = 1e-3

	// DoCommand keys.
	loadKey                    = "load"
	moveGripperKey             = "move_gripper"
	getGripperKey              = "get_gripper"
	gripperPositionKey         = "gripper_position"
	setAcckey                  = "set_acceleration"
	setSpeedKey                = "set_speed"
	grabVacuumKey              = "grab_vacuum"
	openVacuumKey              = "open_vacuum"
	clearErrorKey              = "clear_error"
	getStateKey                = "get_state"
	getErrorKey                = "get_error"
	getVacuumGripperStateKey   = "get_vacuum_state"
	vacuumGripperStateKey      = "vacuum_state"
	gripperLiteActionKey       = "gripper_lite_action"
	setGripperSpeedKey         = "set_gripper_speed"
	getGripperSpeedKey         = "get_gripper_speed"
	gripperSpeedKey            = "gripper_speed"
	enterManualModeKey         = "enter_manual_mode"
	exitManualModeKey          = "exit_manual_mode"
	setCollisionSensitivityKey = "set_collision_sensitivity"

	// gripperLiteActionKeys.
	gripperLiteActionOpen     = "open"
	gripperLiteActionClose    = "close"
	gripperLiteActionIsClosed = "is_closed"
	gripperLiteActionStop     = "stop"
)

//go:embed xarm6_kinematics.json
var xArm6modeljson []byte

//go:embed xarm7_kinematics.json
var xArm7modeljson []byte

//go:embed lite6_kinematics.json
var lite6modeljson []byte

//go:embed xarm850_kinematics.json
var xArm850modeljson []byte

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

var modelNameToURDFFile = map[string]string{
	ModelName6DOF: "xarm6.urdf",
	ModelName7DOF: "xarm7.urdf",
	ModelNameLite: "lite6.urdf",
	ModelName850:  "uf850.urdf",
}

var armTo3DModelParts = map[string][]string{
	"lite6": {
		"base_top",
		"base",
		"gripper_mount",
		"lower_forearm",
		"upper_arm",
		"upper_forearm",
		"wrist_link",
	},
	"xArm6": {
		"base_top",
		"base",
		"gripper_mount",
		"lower_forearm",
		"upper_arm",
		"upper_forearm",
		"wrist_link",
	},
}

type xArm struct {
	resource.AlwaysRebuild

	// cmdConn carries arm motion, state queries, and direct TGPIO writes
	// (port 502). gripperConn carries gripper Modbus passthrough traffic
	// (port 503 when reachable, otherwise aliased to cmdConn).
	cmdConn     *modbusConn
	gripperConn *modbusConn

	// state of movement things
	started atomic.Int32 // -1 is off, >= 0 is mode

	name        resource.Name
	conf        *Config
	closed      atomic.Bool
	opMgr       *operation.SingleOperationManager
	logger      logging.Logger
	motion      motion.Service
	trajGen     mlmodel.Service
	proxyServer *http.Server

	// below is all configuration things
	dof    int
	model  referenceframe.Model
	moveHZ float64 // Number of joint positions to send per second

	// TODO: remove this lock and make this not settable
	confLock     sync.Mutex // speed and acceleration are both able to be read/written to, so they need to be protected by a mutex
	speed        float64    // speed=max joint radians per second
	acceleration float64    // acceleration= joint radians per second increase per second

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
	Host                 string         `json:"host"`
	Port                 int            `json:"port,omitempty"`
	Speed                float64        `json:"speed_degs_per_sec,omitempty"`
	Acceleration         float64        `json:"acceleration_degs_per_sec_per_sec,omitempty"`
	MoveHZ               float64        `json:"move_hz,omitempty"`
	Sensitivity          *int           `json:"collision_sensitivity,omitempty"`
	BadJoints            []int          `json:"bad-joints"`
	Motion               string         `json:"motion"`
	UseURDFs             bool           `json:"use_urdfs,omitempty"`
	TrajGen              *TrajGenConfig `json:"trajectory_generator,omitempty"`
	MeshDecimationRatios []float64      `json:"mesh_decimation_ratios,omitempty"`

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

	for i, r := range cfg.MeshDecimationRatios {
		if r < 0 || r > 1 {
			return nil, nil, fmt.Errorf("mesh_decimation_ratios[%d] must be in [0, 1], got %f", i, r)
		}
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

func getModelJSON(modelName string) ([]byte, error) {
	switch modelName {
	case ModelName6DOF:
		return xArm6modeljson, nil
	case ModelNameLite:
		return lite6modeljson, nil
	case ModelName7DOF:
		return xArm7modeljson, nil
	case ModelName850:
		return xArm850modeljson, nil
	default:
		return nil, fmt.Errorf("no kinematics information for xarm of model %s", modelName)
	}
}

// MakeModelFrame returns the kinematics model of the xarm arm, which has all Frame information.
func MakeModelFrame(
	modelName string, badJoints []int, current []referenceframe.Input, useURDFs bool, meshDecimationRatios []float64, logger logging.Logger,
) (referenceframe.Model, error) {
	var cfg *referenceframe.ModelConfigJSON
	if useURDFs {
		parsed, err := makeModelFrameFromURDF(modelName, meshDecimationRatios)
		if err != nil {
			return nil, err
		}
		cfg = parsed.ModelConfig()
	} else {
		jsonData, err := getModelJSON(modelName)
		if err != nil {
			return nil, err
		}
		if len(jsonData) == 0 {
			return nil, referenceframe.ErrNoModelInformation
		}
		cfg = &referenceframe.ModelConfigJSON{OriginalFile: &referenceframe.ModelFile{Bytes: jsonData, Extension: "json"}}
		if err := json.Unmarshal(jsonData, cfg); err != nil {
			return nil, errors.Wrap(err, "failed to unmarshal json file")
		}
	}

	for _, j := range badJoints {
		now := utils.RadToDeg(current[j])
		cfg.Joints[j].Min = now - 1
		cfg.Joints[j].Max = now + 1
		logger.Infof("locking joint %d to %v", j, now)
	}

	return cfg.ParseConfig(modelName)
}

func makeModelFrameFromURDF(modelName string, meshDecimationRatios []float64) (referenceframe.Model, error) {
	urdfFile, ok := modelNameToURDFFile[modelName]
	if !ok {
		return nil, fmt.Errorf("no URDF file for xarm model %s", modelName)
	}
	moduleRoot := os.Getenv("VIAM_MODULE_ROOT")
	path := fmt.Sprintf("%s/arm/%s", moduleRoot, urdfFile)
	return referenceframe.ParseModelXMLFile(path, modelName, meshDecimationRatios)
}

// newxArm returns a new xArm of the specified modelName.
func newxArm(ctx context.Context, conf resource.Config, logger logging.Logger, modelName string, deps resource.Dependencies) (arm.Arm, error) {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

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
	x.cmdConn = newModbusConn(newConf.host(), logger, func() { x.started.Store(-1) })
	x.gripperConn = x.cmdConn // overwritten below if port 503 connects

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

	err = x.start(ctx, false)
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

	if newConf.UseURDFs && len(newConf.MeshDecimationRatios) == 0 {
		numJoints := 7
		if modelName == ModelName6DOF || modelName == ModelNameLite {
			numJoints = 6
		}
		newConf.MeshDecimationRatios = make([]float64, numJoints)
		for i := range newConf.MeshDecimationRatios {
			newConf.MeshDecimationRatios[i] = 0.1
		}
	}

	x.model, err = MakeModelFrame(modelName, newConf.BadJoints, current, newConf.UseURDFs, newConf.MeshDecimationRatios, logger)
	if err != nil {
		return nil, err
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
			return nil, err
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

func threeDMeshFromName(model, name string) (commonpb.Mesh, error) {
	moduleRoot := os.Getenv("VIAM_MODULE_ROOT")
	path := fmt.Sprintf("%s/arm/3d_models/%s/%s.glb", moduleRoot, model, name)

	// the model path is safe because it is constructed from the module root and the model and name and has no user input
	// #nosec G304
	glb, err := os.ReadFile(path)
	if err != nil {
		return commonpb.Mesh{}, err
	}
	return commonpb.Mesh{
		Mesh:        glb,
		ContentType: "model/gltf-binary",
	}, nil
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

func (x *xArm) Get3DModels(ctx context.Context, extra map[string]any) (map[string]*commonpb.Mesh, error) {
	models := make(map[string]*commonpb.Mesh)
	armModelParts := armTo3DModelParts[x.model.Name()]
	if armModelParts == nil {
		return models, nil
	}

	for _, modelPart := range armModelParts {
		modelPartMesh, err := threeDMeshFromName(x.model.Name(), modelPart)
		if err != nil {
			return nil, err
		}
		models[modelPart] = &modelPartMesh
	}

	return models, nil
}

func (x *xArm) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return x.model, nil
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
	if val, ok := cmd[setCollisionSensitivityKey]; ok {
		sensitivity, err := utils.AssertType[float64](val)
		if err != nil {
			return nil, err
		}
		// The arm only accepts whole-number sensitivity levels in [0, 5];
		// 0 disables collision detection, 5 is the most sensitive.
		level := int(sensitivity)
		if float64(level) != sensitivity || level < 0 || level > 5 {
			return nil, fmt.Errorf("collision sensitivity must be an integer between 0 and 5, got %v", val)
		}
		// Refuse while moving: changing the threshold mid-trajectory could race
		// with the in-flight motion commands, so callers must set it between moves.
		moving, err := x.IsMoving(ctx)
		if err != nil {
			return nil, err
		}
		if moving {
			return nil, errors.New("cannot set collision sensitivity while the arm is moving")
		}
		if err := x.setCollisionDetectionSensitivity(ctx, level); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd[grabVacuumKey]; ok {
		_, ok := cmd[grabVacuumKey].(bool)
		if !ok {
			return nil, errors.New("could not read grab_vacuum")
		}
		if err := x.grabVacuum(ctx); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd[openVacuumKey]; ok {
		_, ok := cmd[openVacuumKey].(bool)
		if !ok {
			return nil, errors.New("could not read close_vacuum")
		}
		if err := x.openVacuum(ctx); err != nil {
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
		res, err := x.getVacuumStatus(ctx)
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

	if !validCommand {
		return nil, errors.New("command not found")
	}
	return resp, nil
}

func (x *xArm) Name() resource.Name {
	return x.name
}

func (x *xArm) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}
