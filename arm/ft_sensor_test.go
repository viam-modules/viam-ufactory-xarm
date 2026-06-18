package arm

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/test"
)

// fakeArm embeds arm.Arm so only DoCommand is implemented; any other call panics,
// which is fine because the sensor only uses DoCommand.
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

	_, err := s.DoCommand(context.Background(), map[string]any{"tare": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fa.lastCmd[ftSensorZeroKey], test.ShouldEqual, true)

	_, err = s.DoCommand(context.Background(), map[string]any{"enable": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fa.lastCmd[setFTSensorEnableKey], test.ShouldEqual, true)
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
