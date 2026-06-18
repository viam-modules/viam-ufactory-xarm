package arm

import (
	"context"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/utils"
)

// FTSensorModel is the model for the ufactory wrist-mounted 6-axis Force/Torque sensor.
var FTSensorModel = family.WithModel("ft_sensor")

// FTSensorConfig is the config for the F/T sensor. It depends on an arm, which owns
// the controller connection.
type FTSensorConfig struct {
	Arm string `json:"arm"`
	// EnableOnStart sends set_ft_sensor_enable=true once at construction. Default
	// false so the controller's persisted enable state (e.g. set in UFactory Studio)
	// stays authoritative; non-Studio users set this true so reads work without a
	// manual DoCommand.
	EnableOnStart bool `json:"enable_on_start,omitempty"`
}

// Validate ensures the arm dependency is set and returns it as a required dependency.
func (cfg *FTSensorConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "arm")
	}
	return []string{cfg.Arm}, nil, nil
}

func init() {
	resource.RegisterComponent(
		sensor.API,
		FTSensorModel,
		resource.Registration[sensor.Sensor, *FTSensorConfig]{
			Constructor: newFTSensor,
		})
}

type ftSensor struct {
	resource.AlwaysRebuild

	name   resource.Name
	arm    arm.Arm
	logger logging.Logger
}

func newFTSensor(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	newConf, err := resource.NativeConfig[*FTSensorConfig](conf)
	if err != nil {
		return nil, err
	}
	s := &ftSensor{
		name:   conf.ResourceName(),
		logger: logger,
	}
	s.arm, err = arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}
	if newConf.EnableOnStart {
		if _, err := s.arm.DoCommand(ctx, map[string]any{setFTSensorEnableKey: true}); err != nil {
			return nil, errors.Wrap(err, "failed to enable F/T sensor on start")
		}
	}
	return s, nil
}

// Readings returns the latest F/T values using UR-compatible keys.
func (s *ftSensor) Readings(ctx context.Context, extra map[string]any) (map[string]any, error) {
	res, err := s.arm.DoCommand(ctx, map[string]any{getFTSensorDataKey: true})
	if err != nil {
		return nil, err
	}
	data, ok := res[ftSensorDataKey].(map[string]any)
	if !ok {
		return nil, errors.Errorf("arm did not return %s map, got %v", ftSensorDataKey, res)
	}
	return data, nil
}

// DoCommand supports {"tare": true} to zero the sensor and {"enable": <bool>} to
// enable/disable it.
func (s *ftSensor) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	if _, ok := cmd["tare"]; ok {
		return s.arm.DoCommand(ctx, map[string]any{ftSensorZeroKey: true})
	}
	if val, ok := cmd["enable"]; ok {
		return s.arm.DoCommand(ctx, map[string]any{setFTSensorEnableKey: val})
	}
	return map[string]any{}, nil
}

func (s *ftSensor) Name() resource.Name {
	return s.name
}

func (s *ftSensor) Close(ctx context.Context) error {
	return nil
}

func (s *ftSensor) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}
