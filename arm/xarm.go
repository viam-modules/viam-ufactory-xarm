// Package arm defines the arm that a robot uses to manipulate objects.
package arm

import (
	"context"
	// for embedding model file.
	_ "embed"
	"fmt"
	"net"
	"sync"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

const (
	modelName6DOF = "xArm6" // ModelName6DOF is the name of a UFactory xArm 6
	modelName7DOF = "xArm7" // ModelName7DOF is the name of a UFactory xArm 7
	modelNameLite = "lite6" // ModelNameLite is the name of a UFactory Lite 6

	defaultSpeed  = 50.  // degrees per second
	defaultAccel  = 100. // degrees per second per second
	defaultPort   = "502"
	defaultMoveHz = 100. // Don't change this

	interwaypointAccel = 600. // degrees per second per second. All xarms max out at 1145
)

//go:embed xarm6_kinematics.json
var xArm6modeljson []byte

//go:embed xarm7_kinematics.json
var xArm7modeljson []byte

//go:embed lite6_kinematics.json
var lite6modeljson []byte

var (
	// XArm6Model defines the resource.Model for the xArm6.
	XArm6Model = resource.NewModel("viam", "ufactory", modelName6DOF)
	// XArm7Model defines the resource.Model for the xArm7.
	XArm7Model = resource.NewModel("viam", "ufactory", modelName7DOF)
	// XArmLite6Model defines the resource.Model for the lite6.
	XArmLite6Model = resource.NewModel("viam", "ufactory", modelNameLite)
)

type xArm struct {
	resource.Named
	dof      int
	tid      uint16
	moveHZ   float64 // Number of joint positions to send per second
	moveLock sync.Mutex
	model    referenceframe.Model
	started  bool
	opMgr    *operation.SingleOperationManager
	logger   logging.Logger

	mu           sync.RWMutex
	conn         net.Conn
	speed        float64 // speed=max joint radians per second
	acceleration float64 // acceleration= joint radians per second increase per second
}

// Config is used for converting config attributes.
type Config struct {
	Host         string  `json:"host"`
	Port         int     `json:"port,omitempty"`
	Speed        float32 `json:"speed_degs_per_sec,omitempty"`
	Acceleration float32 `json:"acceleration_degs_per_sec_per_sec,omitempty"`
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

// Validate validates the config.
func (cfg *Config) Validate(path string) ([]string, error) {
	if cfg.Host == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	return []string{}, nil
}

// MakeModelFrame returns the kinematics model of the xarm arm, which has all Frame information.
func MakeModelFrame(name, modelName string) (referenceframe.Model, error) {
	switch modelName {
	case modelName6DOF:
		return referenceframe.UnmarshalModelJSON(xArm6modeljson, name)
	case modelNameLite:
		return referenceframe.UnmarshalModelJSON(lite6modeljson, name)
	case modelName7DOF:
		return referenceframe.UnmarshalModelJSON(xArm7modeljson, name)
	default:
		return nil, fmt.Errorf("no kinematics information for xarm of model %s", modelName)
	}
}

// newxArm returns a new xArm of the specified modelName.
func newxArm(ctx context.Context, conf resource.Config, logger logging.Logger, modelName string) (arm.Arm, error) {
	model, err := MakeModelFrame(conf.Name, modelName)
	if err != nil {
		return nil, err
	}

	xA := xArm{
		Named:   conf.ResourceName().AsNamed(),
		dof:     len(model.DoF()),
		tid:     0,
		moveHZ:  defaultMoveHz,
		model:   model,
		started: false,
		opMgr:   operation.NewSingleOperationManager(),
		logger:  logger,
	}

	if err := xA.Reconfigure(ctx, nil, conf); err != nil {
		return nil, err
	}

	return &xA, nil
}

// Reconfigure atomically reconfigures this arm in place based on the new config.
func (x *xArm) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return err
	}

	if newConf.Host == "" {
		return errors.New("xArm host not set")
	}

	speed := newConf.Speed
	if speed == 0 {
		speed = defaultSpeed
	}

	acceleration := newConf.Acceleration
	if acceleration == 0 {
		acceleration = defaultAccel
	}
	if acceleration < 0 {
		return fmt.Errorf("given acceleration %f cannot be negative", acceleration)
	}

	port := fmt.Sprintf("%d", newConf.Port)
	if newConf.Port == 0 {
		port = defaultPort
	}

	x.mu.Lock()
	defer x.mu.Unlock()

	newAddr := net.JoinHostPort(newConf.Host, port)
	if x.conn == nil || x.conn.RemoteAddr().String() != newAddr {
		// Need a new or replacement connection
		var d net.Dialer
		newConn, err := d.DialContext(ctx, "tcp", newAddr)
		if err != nil {
			return err
		}
		if x.conn != nil {
			if err := x.conn.Close(); err != nil {
				x.logger.CWarnw(ctx, "error closing old connection but will continue with reconfiguration", "error", err)
			}
		}
		x.conn = newConn

		if err := x.start(ctx); err != nil {
			return errors.Wrap(err, "failed to start on reconfigure")
		}
	}

	x.acceleration = utils.DegToRad(float64(acceleration))
	x.speed = utils.DegToRad(float64(speed))
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

	if _, ok := cmd["setup_gripper"]; ok {
		if err := x.enableGripper(ctx); err != nil {
			return nil, err
		}
		if err := x.setGripperMode(ctx, false); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if val, ok := cmd["move_gripper"]; ok {
		position, ok := val.(float64)
		if !ok || position < -10 || position > 850 {
			return nil, fmt.Errorf("must move gripper to an int between 0 and 840 %v", val)
		}
		if err := x.setGripperPosition(ctx, uint32(position)); err != nil {
			return nil, err
		}
		validCommand = true
	}
	if _, ok := cmd["load"]; ok {
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
		x.speed = utils.DegToRad(speed)
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
		x.acceleration = utils.DegToRad(acceleration)
		validCommand = true
	}

	if !validCommand {
		return nil, errors.New("command not found")
	}
	return resp, nil
}
