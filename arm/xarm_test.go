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

func TestConnectionTypeFromCmd(t *testing.T) {
	test.That(t, connectionTypeFromCmd(map[string]any{connectionTypeKey: "contact"}, submodelV1),
		test.ShouldEqual, connectionContact)
	test.That(t, connectionTypeFromCmd(map[string]any{connectionTypeKey: "plugin"}, submodelV2),
		test.ShouldEqual, connectionPlugin)
	test.That(t, connectionTypeFromCmd(map[string]any{}, submodelV2),
		test.ShouldEqual, connectionContact)
	test.That(t, connectionTypeFromCmd(map[string]any{}, submodelV1),
		test.ShouldEqual, connectionPlugin)
	test.That(t, connectionTypeFromCmd(map[string]any{connectionTypeKey: "nonsense"}, submodelV1),
		test.ShouldEqual, connectionPlugin)
}

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
			m, err := MakeModelFrame("", tc.model, nil, nil, false, nil, logger, 0)
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
			m, err := MakeModelFrame("", tc.model, nil, nil, true, nil, logger, 0)
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

	_, err := MakeModelFrame("", ModelName6DOF, nil, nil, true, nil, logger, 0)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestMakeModelFrameURDFUnknownModel(t *testing.T) {
	logger := logging.NewTestLogger(t)

	_, err := MakeModelFrame("", "unknownModel", nil, nil, true, nil, logger, 0)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no kinematics artifact for xarm model")
}

func TestMakeModelFrameWithBadJoints(t *testing.T) {
	logger := logging.NewTestLogger(t)

	// Provide fake current positions for a 6-DOF arm.
	current := make([]referenceframe.Input, 6)
	for i := range current {
		current[i] = 0
	}

	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, false, nil, logger, 0)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, m, test.ShouldNotBeNil)
	test.That(t, len(m.DoF()), test.ShouldEqual, 6)
}

func TestUseURDFsDefaultsFalse(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.UseURDFs, test.ShouldBeFalse)
}

func TestResolveArmKinematicsArtifact(t *testing.T) {
	cases := []struct {
		name             string
		model            string
		detected         detectedArm
		wantURDFBasename string
		wantVariant      string
		wantErr          bool
	}{
		{
			name:             "xArm6 base, no detection",
			model:            ModelName6DOF,
			detected:         detectedArm{},
			wantURDFBasename: "xarm6",
		},
		{
			name:             "xArm6 with 1305 hardware variant",
			model:            ModelName6DOF,
			detected:         detectedArm{armTypeCode: 1305},
			wantURDFBasename: "xarm6_1305",
			wantVariant:      "1305",
		},
		{
			name:             "xArm6 with unknown armTypeCode falls back to base",
			model:            ModelName6DOF,
			detected:         detectedArm{armTypeCode: 9999},
			wantURDFBasename: "xarm6",
		},
		{
			name:             "xArm850 base",
			model:            ModelName850,
			detected:         detectedArm{},
			wantURDFBasename: "uf850",
		},
		{
			name:     "unknown model returns error",
			model:    "ghost",
			detected: detectedArm{},
			wantErr:  true,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got, err := resolveArmKinematicsArtifact(tc.model, tc.detected)
			if tc.wantErr {
				test.That(t, err, test.ShouldNotBeNil)
				return
			}
			test.That(t, err, test.ShouldBeNil)
			test.That(t, got.urdfBasename, test.ShouldEqual, tc.wantURDFBasename)
			test.That(t, got.variant, test.ShouldEqual, tc.wantVariant)
			test.That(t, len(got.json), test.ShouldBeGreaterThan, 0)
		})
	}
}

func TestKinematicsArtifactURDFsOnDisk(t *testing.T) {
	dir := armDir()
	for _, base := range armKinematicsBase {
		path := filepath.Join(dir, base.urdfBasename+".urdf")
		_, err := os.Stat(path)
		test.That(t, err, test.ShouldBeNil)
	}
	for _, v := range armKinematicsVariants {
		path := filepath.Join(dir, v.urdfBasename+".urdf")
		_, err := os.Stat(path)
		test.That(t, err, test.ShouldBeNil)
	}
}

func TestMakeModelFrameVariantURDF(t *testing.T) {
	logger := logging.NewTestLogger(t)
	repoRoot := filepath.Dir(armDir())
	t.Setenv("VIAM_MODULE_ROOT", repoRoot)

	m, err := MakeModelFrame("", ModelName6DOF, nil, nil, true, nil, logger, 1305)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, m, test.ShouldNotBeNil)
	test.That(t, len(m.DoF()), test.ShouldEqual, 6)
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
