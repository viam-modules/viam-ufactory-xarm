package arm

import (
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
)

// armDir returns the absolute path to the arm/ directory containing test data.
func armDir() string {
	//nolint:dogsled
	_, file, _, _ := runtime.Caller(0)
	return filepath.Dir(file)
}

func TestMakeModelFrameJSON(t *testing.T) {
	logger := logging.NewTestLogger(t)

	tests := []struct {
		name     string
		model    string
		expected int
	}{
		{"xArm6", ModelName6DOF, 6},
		{"xArm7", ModelName7DOF, 7},
		{"lite6", ModelNameLite, 6},
		{"xArm850", ModelName850, 6},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			m, err := MakeModelFrame("", tc.model, nil, nil, false, nil, logger)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, m, test.ShouldNotBeNil)
			test.That(t, len(m.DoF()), test.ShouldEqual, tc.expected)
			test.That(t, m.Name(), test.ShouldEqual, tc.model)
		})
	}
}

func TestMakeModelFrameURDF(t *testing.T) {
	logger := logging.NewTestLogger(t)

	// Point VIAM_MODULE_ROOT to the repo root (parent of arm/).
	repoRoot := filepath.Dir(armDir())
	t.Setenv("VIAM_MODULE_ROOT", repoRoot)

	tests := []struct {
		name     string
		model    string
		expected int
	}{
		{"xArm6", ModelName6DOF, 6},
		{"xArm7", ModelName7DOF, 7},
		{"lite6", ModelNameLite, 6},
		{"xArm850", ModelName850, 6},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			m, err := MakeModelFrame("", tc.model, nil, nil, true, nil, logger)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, m, test.ShouldNotBeNil)
			test.That(t, len(m.DoF()), test.ShouldEqual, tc.expected)
		})
	}
}

func TestMakeModelFrameURDFMissingEnv(t *testing.T) {
	logger := logging.NewTestLogger(t)

	// Ensure VIAM_MODULE_ROOT points to a nonexistent directory.
	t.Setenv("VIAM_MODULE_ROOT", "/nonexistent/path")

	_, err := MakeModelFrame("", ModelName6DOF, nil, nil, true, nil, logger)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestMakeModelFrameURDFUnknownModel(t *testing.T) {
	logger := logging.NewTestLogger(t)

	_, err := MakeModelFrame("", "unknownModel", nil, nil, true, nil, logger)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no URDF file for xarm model")
}

func TestMakeModelFrameWithBadJoints(t *testing.T) {
	logger := logging.NewTestLogger(t)

	// Provide fake current positions for a 6-DOF arm.
	current := make([]referenceframe.Input, 6)
	for i := range current {
		current[i] = 0
	}

	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, false, nil, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, m, test.ShouldNotBeNil)
	test.That(t, len(m.DoF()), test.ShouldEqual, 6)
}

func TestUseURDFsDefaultsFalse(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.UseURDFs, test.ShouldBeFalse)
}

func TestModelNameToURDFFileMapping(t *testing.T) {
	// Verify every model has a URDF mapping.
	for _, model := range []string{ModelName6DOF, ModelName7DOF, ModelNameLite, ModelName850} {
		_, ok := modelNameToURDFFile[model]
		test.That(t, ok, test.ShouldBeTrue)
	}

	// Verify the URDF files actually exist on disk.
	dir := armDir()
	for _, urdfFile := range modelNameToURDFFile {
		path := filepath.Join(dir, urdfFile)
		_, err := os.Stat(path)
		test.That(t, err, test.ShouldBeNil)
	}
}

func TestMoveOptions(t *testing.T) {
	logger := logging.NewTestLogger(t)

	x := &xArm{
		speed:        utils.DegToRad(defaultSpeed),
		acceleration: utils.DegToRad(defaultAccel),
		moveHZ:       defaultMoveHz,
		logger:       logger,
	}

	base := x.moveOptions(nil, nil)
	test.That(t, base.speed, test.ShouldEqual, x.speed)
	test.That(t, base.acceleration, test.ShouldEqual, x.acceleration)
	test.That(t, base.moveHZ, test.ShouldEqual, x.moveHZ)

	mo := x.moveOptions(nil, map[string]any{"acceleration_r": 2.5})
	test.That(t, mo.speed, test.ShouldEqual, base.speed)
	test.That(t, mo.acceleration, test.ShouldEqual, 2.5)
	test.That(t, mo.moveHZ, test.ShouldEqual, base.moveHZ)

	mo = x.moveOptions(nil, map[string]any{"acceleration_r": 500.0})
	test.That(t, mo.speed, test.ShouldEqual, base.speed)
	test.That(t, mo.acceleration, test.ShouldEqual, utils.DegToRad(maxAccel))
	test.That(t, mo.moveHZ, test.ShouldEqual, base.moveHZ)

	mo = x.moveOptions(nil, map[string]any{"speed_r": 500.0})
	test.That(t, mo.speed, test.ShouldEqual, utils.DegToRad(maxSpeed))
	test.That(t, mo.acceleration, test.ShouldEqual, base.acceleration)
	test.That(t, mo.moveHZ, test.ShouldEqual, base.moveHZ)

	mo = x.moveOptions(nil, map[string]any{"speed_d": 90.0})
	test.That(t, mo.speed, test.ShouldEqual, math.Pi/2)
	test.That(t, mo.acceleration, test.ShouldEqual, base.acceleration)
	test.That(t, mo.moveHZ, test.ShouldEqual, base.moveHZ)

	mo = x.moveOptions(nil, map[string]any{"speed_d": 90})
	test.That(t, mo.speed, test.ShouldEqual, math.Pi/2)
	test.That(t, mo.acceleration, test.ShouldEqual, base.acceleration)
	test.That(t, mo.moveHZ, test.ShouldEqual, base.moveHZ)
}

func TestFTReadingsMap(t *testing.T) {
	vals := []float64{-0.987, -2.923, -18.356, -0.0012, -0.0914, 0.00698}
	m := ftReadingsMap(vals)
	test.That(t, m["Fx_N"], test.ShouldEqual, -0.987)
	test.That(t, m["Fy_N"], test.ShouldEqual, -2.923)
	test.That(t, m["Fz_N"], test.ShouldEqual, -18.356)
	test.That(t, m["TRx_Nm"], test.ShouldEqual, -0.0012)
	test.That(t, m["TRy_Nm"], test.ShouldEqual, -0.0914)
	test.That(t, m["TRz_Nm"], test.ShouldEqual, 0.00698)
	test.That(t, len(m), test.ShouldEqual, 6)
}
