package arm

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/test"
)

// fakeArm embeds arm.Arm so only DoCommand needs implementing.
type fakeArm struct {
	arm.Arm
	lastCmd map[string]any
	resp    map[string]any
	err     error
}

func (f *fakeArm) DoCommand(_ context.Context, cmd map[string]any) (map[string]any, error) {
	f.lastCmd = cmd
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
