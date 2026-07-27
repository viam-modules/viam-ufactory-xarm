package arm

import (
	"context"
	"errors"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/test"
)

// fakeArm embeds arm.Arm so only DoCommand needs implementing. Set doFn to vary
// the response by command (e.g. simulate a stream that wakes up after enable);
// otherwise it returns the static resp/err.
type fakeArm struct {
	arm.Arm
	lastCmd map[string]any
	resp    map[string]any
	err     error
	doFn    func(cmd map[string]any) (map[string]any, error)
}

func (f *fakeArm) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	f.lastCmd = cmd
	if f.doFn != nil {
		return f.doFn(cmd)
	}
	return f.resp, f.err
}

func TestFTSensorReadings(t *testing.T) {
	fa := &fakeArm{resp: map[string]any{
		ftSensorDataKey: map[string]any{
			"Fx_N": -0.987, "Fy_N": -2.923, "Fz_N": -18.356,
			"TRx_Nm": -0.0012, "TRy_Nm": -0.0914, "TRz_Nm": 0.00698,
		},
	}}
	s := &ftSensor{arm: fa}

	readings, err := s.Readings(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, readings["Fz_N"], test.ShouldEqual, -18.356)
	test.That(t, fa.lastCmd[getFTSensorDataKey], test.ShouldEqual, true)
}

func TestFTSensorDoCommandTare(t *testing.T) {
	fa := &fakeArm{resp: map[string]any{}}
	s := &ftSensor{arm: fa}

	_, err := s.DoCommand(context.Background(), map[string]any{tareKey: true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fa.lastCmd[ftSensorZeroKey], test.ShouldEqual, true)
}

func TestFTSensorReadingsSelfHeal(t *testing.T) {
	// Stream starts disabled (all-zeros); a good read appears once enable fires.
	enabled := false
	zero := map[string]any{ftSensorDataKey: ftReadingsMap([]float64{0, 0, 0, 0, 0, 0})}
	live := map[string]any{ftSensorDataKey: ftReadingsMap([]float64{-0.21, 0.14, -0.3, 0, 0, 0})}
	fa := &fakeArm{doFn: func(cmd map[string]any) (map[string]any, error) {
		if _, ok := cmd[ftSensorEnableKey]; ok {
			enabled = true
			return map[string]any{}, nil
		}
		if enabled {
			return live, nil
		}
		return zero, nil
	}}
	s := &ftSensor{arm: fa, logger: logging.NewTestLogger(t)}

	readings, err := s.Readings(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, enabled, test.ShouldBeTrue)
	test.That(t, readings["Fx_N"], test.ShouldEqual, -0.21)
}

func TestFTSensorReadingsOverloadSurfaced(t *testing.T) {
	// Stream stuck all-zero and enable fails (faulted/overloaded sensor).
	zero := map[string]any{ftSensorDataKey: ftReadingsMap([]float64{0, 0, 0, 0, 0, 0})}
	fa := &fakeArm{doFn: func(cmd map[string]any) (map[string]any, error) {
		if _, ok := cmd[ftSensorEnableKey]; ok {
			return nil, errors.New("controller error 18")
		}
		return zero, nil
	}}
	s := &ftSensor{arm: fa, logger: logging.NewTestLogger(t)}

	_, err := s.Readings(context.Background(), nil)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "power-cycle")
}

func TestFTSensorDoCommandClearError(t *testing.T) {
	fa := &fakeArm{resp: map[string]any{}}
	s := &ftSensor{arm: fa}

	_, err := s.DoCommand(context.Background(), map[string]any{clearErrorKey: true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fa.lastCmd[clearErrorKey], test.ShouldEqual, true)
}

func TestFTSensorEnablesOnStartup(t *testing.T) {
	fa := &fakeArm{resp: map[string]any{}}
	deps := resource.Dependencies{arm.Named("myarm"): fa}
	conf := resource.Config{
		Name:                "ft",
		API:                 sensor.API,
		ConvertedAttributes: &FTSensorConfig{Arm: "myarm"},
	}

	_, err := newFTSensor(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fa.lastCmd[ftSensorEnableKey], test.ShouldEqual, true)
}

func TestFTSensorConfigValidate(t *testing.T) {
	cfg := &FTSensorConfig{}
	_, _, err := cfg.Validate("path")
	test.That(t, err, test.ShouldNotBeNil)

	cfg.Arm = "myarm"
	deps, _, err := cfg.Validate("path")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"myarm"})
}
