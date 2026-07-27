package arm

import (
	"context"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/utils"
)

// FTSensorModel is the model for the ufactory wrist-mounted 6-axis Force/Torque sensor.
var FTSensorModel = family.WithModel("ft_sensor")

const tareKey = "tare"

// ftReenableCooldown bounds how often Readings will retry an enable when the
// stream is delivering all-zeros. It heals a disabled stream on the first zero
// read, but a faulted/overloaded sensor (which throws controller error 18 on
// enable) is only retried once per cooldown instead of on every poll.
const ftReenableCooldown = 10 * time.Second

// ftEnableSettle bounds how long Readings waits for the stream to produce real
// data after a self-heal enable. Measured settle is a few ms on-LAN; the budget
// is generous for slower/remote links. A genuinely stuck stream falls through to
// the all-zero error after this.
const ftEnableSettle = 250 * time.Millisecond

// ftEnablePollInterval is the pause between reads while polling for the stream
// to come up within ftEnableSettle.
const ftEnablePollInterval = 5 * time.Millisecond

func ftReadingsMap(vals []float64) map[string]any {
	return map[string]any{
		"Fx_N":   vals[0],
		"Fy_N":   vals[1],
		"Fz_N":   vals[2],
		"TRx_Nm": vals[3],
		"TRy_Nm": vals[4],
		"TRz_Nm": vals[5],
	}
}

// FTSensorConfig is the config for the F/T sensor.
type FTSensorConfig struct {
	Arm string `json:"arm"`
}

// Validate ensures the arm dependency is set.
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

	lastReenableNano atomic.Int64
}

func newFTSensor(_ context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
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
	return s, nil
}

func (s *ftSensor) Readings(ctx context.Context, extra map[string]any) (map[string]any, error) {
	data, err := s.readOnce(ctx)
	if err != nil || !ftAllZero(data) {
		return data, err
	}

	// The stream is delivering all-zeros (never a real reading — there is always
	// noise), so it's disabled or faulted. The gripper re-inits on every move; a
	// read-only sensor has no such path, so self-heal here. Rate-limited via
	// CompareAndSwap so an overloaded sensor (enable throws controller error 18)
	// is retried at most once per cooldown, not on every poll.
	now := time.Now().UnixNano()
	last := s.lastReenableNano.Load()
	if now-last > ftReenableCooldown.Nanoseconds() && s.lastReenableNano.CompareAndSwap(last, now) {
		if _, enableErr := s.arm.DoCommand(ctx, map[string]any{ftSensorEnableKey: true}); enableErr != nil {
			s.logger.Debugf("F/T self-heal enable failed (sensor may be overloaded): %v", enableErr)
		} else {
			data, err = s.readAfterEnable(ctx)
			if err != nil {
				return nil, err
			}
		}
	}

	if ftAllZero(data) {
		return nil, errors.Errorf(
			"F/T sensor returning all zeros after re-enable; sensor disabled/disconnected, " +
				"or overloaded (power-cycle the controller if get_ft_sensor_error is 64-71)")
	}
	return data, nil
}

// readAfterEnable polls the stream after a self-heal enable. The controller
// needs a few ms to start producing real data — the first read after enable is
// always zero (measured ~3ms, worst ~6ms on-LAN) — so a single re-read would
// misreport a healthy re-enable as a stuck sensor. Poll until non-zero or the
// settle budget elapses; a genuinely stuck stream then falls through to the
// all-zero error.
func (s *ftSensor) readAfterEnable(ctx context.Context) (map[string]any, error) {
	deadline := time.Now().Add(ftEnableSettle)
	for {
		data, err := s.readOnce(ctx)
		if err != nil || !ftAllZero(data) || time.Now().After(deadline) {
			return data, err
		}
		if !utils.SelectContextOrWait(ctx, ftEnablePollInterval) {
			return data, ctx.Err()
		}
	}
}

func (s *ftSensor) readOnce(ctx context.Context) (map[string]any, error) {
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

// ftAllZero reports whether every axis is exactly 0.0 — the signature of a
// disabled/faulted stream, since a live sensor always reads some noise.
func ftAllZero(data map[string]any) bool {
	for _, v := range data {
		if f, ok := v.(float64); ok && f != 0 {
			return false
		}
	}
	return true
}

func (s *ftSensor) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	if _, ok := cmd[tareKey]; ok {
		return s.arm.DoCommand(ctx, map[string]any{ftSensorZeroKey: true})
	}
	// clear_error forwards to the arm's controller error-clear. Note: this only
	// clears the controller error box; a sensor overload latch (get_ft_sensor_error
	// 64-71) can only be cleared by power-cycling the controller.
	if _, ok := cmd[clearErrorKey]; ok {
		return s.arm.DoCommand(ctx, map[string]any{clearErrorKey: true})
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
