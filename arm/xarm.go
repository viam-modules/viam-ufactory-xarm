// Package arm defines the arm that a robot uses to manipulate objects.
package arm

import (
	"context"
	_ "embed" // for embedding model file.
	"encoding/json"
	"fmt"
	"net"
	"sync"
	"sync/atomic"

	"github.com/pkg/errors"
	"go.uber.org/multierr"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

const (
	defaultSpeed  = 50.  // degrees per second
	defaultAccel  = 100. // degrees per second per second
	defaultPort   = 502
	defaultMoveHz = 100. // Don't change this

	interwaypointAccel = 600. // degrees per second per second. All xarms max out at 1145
)

//go:embed xarm6_kinematics.json
var xArm6modeljson []byte

//go:embed xarm7_kinematics.json
var xArm7modeljson []byte

//go:embed lite6_kinematics.json
var lite6modeljson []byte

const (
	// ModelName6DOF is the name of a UFactory xArm 6.
	ModelName6DOF = "xArm6"
	// ModelName7DOF is the name of a UFactory xArm 7.
	ModelName7DOF = "xArm7"
	// ModelNameLite is the name of a UFactory Lite 6.
	ModelNameLite = "lite6"
)

var (
	family = resource.ModelNamespace("viam").WithFamily("ufactory")

	// XArm6Model defines the resource.Model for the xArm6.
	XArm6Model = family.WithModel(ModelName6DOF)
	// XArm7Model defines the resource.Model for the xArm7.
	XArm7Model = family.WithModel(ModelName7DOF)
	// XArmLite6Model defines the resource.Model for the lite6.
	XArmLite6Model = family.WithModel(ModelNameLite)
)

type xArm struct {
	resource.AlwaysRebuild

	moveLock sync.Mutex

	// state of movement things
	started bool
	tid     uint16

	name   resource.Name
	conf   *Config
	conn   net.Conn
	closed atomic.Bool
	opMgr  *operation.SingleOperationManager
	logger logging.Logger

	// below is all configuration things
	dof    int
	model  referenceframe.Model
	moveHZ float64 // Number of joint positions to send per second

	confLock     sync.Mutex // speed and acceleration are both able to be read/written to, so they need to be protected by a mutex
	speed        float64    // speed=max joint radians per second
	acceleration float64    // acceleration= joint radians per second increase per second
}

func init() {
	for _, model := range []resource.Model{XArm6Model, XArm7Model, XArmLite6Model} {
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
				_ resource.Dependencies,
				conf resource.Config,
				logger logging.Logger,
			) (arm.Arm, error) {
				return newxArm(ctx, conf, logger, model.Name)
			},
		},
	)
}

// Config is used for converting config attributes.
type Config struct {
	Host         string  `json:"host"`
	Port         int     `json:"port,omitempty"`
	Speed        float32 `json:"speed_degs_per_sec,omitempty"`
	Acceleration float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
	BadJoints    []int   `json:"bad-joints"`
}

// Validate validates the config.
func (cfg *Config) Validate(path string) ([]string, error) {
	if cfg.Host == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	if cfg.Acceleration < 0 {
		return nil, fmt.Errorf("given acceleration %f cannot be negative", cfg.Acceleration)
	}

	return []string{}, nil
}

func (cfg *Config) speed() float32 {
	if cfg.Speed == 0 {
		return defaultSpeed
	}
	return cfg.Speed
}

func (cfg *Config) acceleration() float32 {
	if cfg.Acceleration == 0 {
		return defaultAccel
	}
	return cfg.Acceleration
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
	default:
		return nil, fmt.Errorf("no kinematics information for xarm of model %s", modelName)
	}
}

// MakeModelFrame returns the kinematics model of the xarm arm, which has all Frame information.
func MakeModelFrame(modelName string, badJoints []int, current []referenceframe.Input, logger logging.Logger) (referenceframe.Model, error) {
	jsonData, err := getModelJSON(modelName)
	if err != nil {
		return nil, err
	}

	// empty data probably means that the robot component has no model information
	if len(jsonData) == 0 {
		return nil, referenceframe.ErrNoModelInformation
	}

	m := &referenceframe.ModelConfig{OriginalFile: &referenceframe.ModelFile{Bytes: jsonData, Extension: "json"}}

	err = json.Unmarshal(jsonData, m)
	if err != nil {
		return nil, errors.Wrap(err, "failed to unmarshal json file")
	}

	for _, j := range badJoints {
		now := utils.RadToDeg(current[j].Value)
		m.Joints[j].Min = now - 1
		m.Joints[j].Max = now + 1
		logger.Infof("locking joint %d to %v", j, now)
	}

	return m.ParseConfig(modelName)
}

// newxArm returns a new xArm of the specified modelName.
func newxArm(ctx context.Context, conf resource.Config, logger logging.Logger, modelName string) (arm.Arm, error) {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	return NewXArm(ctx, conf.ResourceName(), newConf, logger, modelName)
}

// NewXArm creates a new x arm connection.
func NewXArm(ctx context.Context, name resource.Name, newConf *Config, logger logging.Logger, modelName string) (arm.Arm, error) {
	x := xArm{
		name:    name,
		conf:    newConf,
		tid:     0,
		moveHZ:  defaultMoveHz,
		started: false,
		opMgr:   operation.NewSingleOperationManager(),
		logger:  logger,

		acceleration: utils.DegToRad(float64(newConf.acceleration())),
		speed:        utils.DegToRad(float64(newConf.speed())),
	}

	err := x.connect(ctx)
	if err != nil {
		return nil, err
	}

	current := []referenceframe.Input{}
	if len(newConf.BadJoints) > 0 {
		x.dof = newConf.maxBadJoint() + 1
		current, err = x.CurrentInputs(ctx)
		if err != nil {
			return nil, multierr.Combine(err, x.Close(ctx))
		}
	}

	x.model, err = MakeModelFrame(modelName, newConf.BadJoints, current, logger)
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

	return &x, nil
}

func (x *xArm) resetConnection() {
	if x.conn == nil {
		return
	}

	err := x.conn.Close()
	if err != nil {
		x.logger.Infof("error closing old socket: %v", err)
	}
	x.conn = nil
}

func (x *xArm) connect(ctx context.Context) error {
	x.resetConnection()

	var d net.Dialer
	var err error

	x.conn, err = d.DialContext(ctx, "tcp", x.conf.host())
	if err != nil {
		return err
	}

	err = x.start(ctx)
	if err != nil {
		err = x.conn.Close()
		if err != nil {
			x.logger.Infof("error closing bad socket: %v", err)
		}
		x.conn = nil
		return errors.Wrap(err, "failed to start xarm")
	}

	return nil
}

func (x *xArm) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return x.JointPositions(ctx, nil)
}

func (x *xArm) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return x.MoveThroughJointPositions(ctx, inputSteps, nil, nil)
}

func (x *xArm) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
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

// ModelFrame returns all the information necessary for including the arm in a FrameSystem.
func (x *xArm) ModelFrame() referenceframe.Model {
	return x.model
}

func (x *xArm) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	resp := map[string]interface{}{}
	validCommand := false

	if val, ok := cmd["move_gripper"]; ok {
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
	if _, ok := cmd["get_gripper"]; ok {
		pos, err := x.getGripperPosition(ctx)
		if err != nil {
			return nil, err
		}
		resp["gripper_position"] = float64(pos)
		validCommand = true
	}

	if _, ok := cmd["load"]; ok {
		if err := x.setupGripper(ctx); err != nil {
			return nil, err
		}
		loadInformation, err := x.getLoad(ctx)
		if err != nil {
			return nil, err
		}
		loadInformationInterface, ok := loadInformation["loads"]
		if !ok {
			return nil, errors.New("could not read loadInformation")
		}
		resp["load"] = loadInformationInterface
		validCommand = true
	}
	if val, ok := cmd["set_speed"]; ok {
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
	if val, ok := cmd["set_acceleration"]; ok {
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
	if _, ok := cmd["grab_vacuum"]; ok {
		_, ok := cmd["grab_vacuum"].(bool)
		if !ok {
			return nil, errors.New("could not read grab_vacuum")
		}
		if err := x.grabVacuum(ctx); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd["open_vacuum"]; ok {
		_, ok := cmd["open_vacuum"].(bool)
		if !ok {
			return nil, errors.New("could not read close_vacuum")
		}
		if err := x.openVacuum(ctx); err != nil {
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
