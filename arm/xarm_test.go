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
			m, err := MakeModelFrame("", tc.model, nil, nil, false, nil, logger, 0, 0, 0)
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
			m, err := MakeModelFrame("", tc.model, nil, nil, true, nil, logger, 0, 0, 0)
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

	_, err := MakeModelFrame("", ModelName6DOF, nil, nil, true, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestMakeModelFrameURDFUnknownModel(t *testing.T) {
	logger := logging.NewTestLogger(t)

	_, err := MakeModelFrame("", "unknownModel", nil, nil, true, nil, logger, 0, 0, 0)
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

	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, m, test.ShouldNotBeNil)
	test.That(t, len(m.DoF()), test.ShouldEqual, 6)
}

// On the URDF path a lock cannot reach the server, since the bytes RDK sends are the URDF off
// disk. It still has to shape the model we return, so this module's own planning refuses to move
// the joint. Losing that quietly is the failure this pins down.
func TestMakeModelFrameBadJointsOnURDFLockOnlyLocally(t *testing.T) {
	logger := logging.NewTestLogger(t)

	repoRoot := filepath.Dir(armDir())
	t.Setenv("VIAM_MODULE_ROOT", repoRoot)

	current := make([]referenceframe.Input, 6)
	current[2] = utils.DegToRad(30)

	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, true, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldBeNil)

	locked := m.DoF()[2]
	test.That(t, utils.RadToDeg(locked.Min), test.ShouldAlmostEqual, 29.0, 1e-8)
	test.That(t, utils.RadToDeg(locked.Max), test.ShouldAlmostEqual, 31.0, 1e-8)

	// The document we hand out is still the URDF, so the lock does not travel with it.
	test.That(t, m.ModelConfig().OriginalFile.Extension, test.ShouldEqual, "urdf")
}

// A locked joint also carries the configured speed, so the two edits have to survive each other:
// they are written into the same map entry, and a careless merge drops one of them.
func TestMakeModelFrameLockAndSpeedLimitsCoexist(t *testing.T) {
	logger := logging.NewTestLogger(t)

	const speed, accel = 45.0, 300.0
	current := make([]referenceframe.Input, 6)
	current[2] = utils.DegToRad(30)

	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, false, nil, logger, 0, speed, accel)
	test.That(t, err, test.ShouldBeNil)

	served, err := referenceframe.UnmarshalModelJSON(m.ModelConfig().OriginalFile.Bytes, "")
	test.That(t, err, test.ShouldBeNil)

	locked := served.DoF()[2]
	test.That(t, utils.RadToDeg(locked.Min), test.ShouldAlmostEqual, 29.0, 1e-8)
	test.That(t, utils.RadToDeg(locked.Max), test.ShouldAlmostEqual, 31.0, 1e-8)

	// Every joint still advertises the configured speed, the locked one included.
	vels, accs, ok := referenceframe.TrajectoryLimits(served.DoF())
	test.That(t, ok, test.ShouldBeTrue)
	for i := range vels {
		test.That(t, vels[i], test.ShouldAlmostEqual, utils.DegToRad(speed), 1e-8)
		test.That(t, accs[i], test.ShouldAlmostEqual, utils.DegToRad(accel), 1e-8)
	}
}

// An index nobody can lock used to panic on the way to `cfg.Joints[j]`. Erroring says the same
// thing without taking the module down, and matters more than it looks: a joint is listed here
// because it is broken, so quietly not locking it is the one outcome we cannot have.
func TestMakeModelFrameBadJointsOutOfRange(t *testing.T) {
	logger := logging.NewTestLogger(t)

	current := make([]referenceframe.Input, 6)

	_, err := MakeModelFrame("", ModelName6DOF, []int{99}, current, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "out of range")

	_, err = MakeModelFrame("", ModelName6DOF, []int{-1}, current, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestUseURDFsDefaultsFalse(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.UseURDFs, test.ShouldBeFalse)
}

// The point of publishing limits is that they leave the module, so this checks the bytes RDK
// actually sends rather than the model we happen to be holding.
func TestMakeModelFramePublishesSpeedLimits(t *testing.T) {
	logger := logging.NewTestLogger(t)

	const speed, accel = 45.0, 300.0
	m, err := MakeModelFrame("", ModelName6DOF, nil, nil, false, nil, logger, 0, speed, accel)
	test.That(t, err, test.ShouldBeNil)

	served, err := referenceframe.UnmarshalModelJSON(m.ModelConfig().OriginalFile.Bytes, "")
	test.That(t, err, test.ShouldBeNil)

	// a trajectory generator can use this arm, and every joint carries the configured speed
	vels, accs, ok := referenceframe.TrajectoryLimits(served.DoF())
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, vels, test.ShouldHaveLength, 6)
	for i := range vels {
		test.That(t, vels[i], test.ShouldAlmostEqual, utils.DegToRad(speed), 1e-8)
		test.That(t, accs[i], test.ShouldAlmostEqual, utils.DegToRad(accel), 1e-8)
	}
}

// Without speeds there is nothing to advertise, and the arm must not claim bounds it was never
// given, or the motion service would plan timing against numbers nobody chose.
func TestMakeModelFrameWithoutSpeedsIsUnbounded(t *testing.T) {
	logger := logging.NewTestLogger(t)

	m, err := MakeModelFrame("", ModelName6DOF, nil, nil, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldBeNil)

	_, _, ok := referenceframe.TrajectoryLimits(m.DoF())
	test.That(t, ok, test.ShouldBeFalse)
}

// A locked joint used to be locked only inside this module: the patch went onto the parsed
// config, but the bytes RDK sends were the untouched ones off disk.
func TestMakeModelFrameBadJointsReachTheWire(t *testing.T) {
	logger := logging.NewTestLogger(t)

	current := make([]referenceframe.Input, 6)
	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldBeNil)

	served, err := referenceframe.UnmarshalModelJSON(m.ModelConfig().OriginalFile.Bytes, "")
	test.That(t, err, test.ShouldBeNil)

	locked := served.DoF()[2]
	test.That(t, utils.RadToDeg(locked.Min), test.ShouldAlmostEqual, -1.0, 1e-8)
	test.That(t, utils.RadToDeg(locked.Max), test.ShouldAlmostEqual, 1.0, 1e-8)
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

	m, err := MakeModelFrame("", ModelName6DOF, nil, nil, true, nil, logger, 1305, 0, 0)
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
