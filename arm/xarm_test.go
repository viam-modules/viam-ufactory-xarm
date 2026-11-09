package arm

import (
	"context"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
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

func TestStreamLimitToleranceRatio(t *testing.T) {
	cfg := Config{Host: "localhost"}
	test.That(t, cfg.streamLimitToleranceRatio(), test.ShouldEqual, defaultStreamLimitToleranceRatio)
	_, _, err := cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)

	cfg.StreamLimitToleranceRatio = 1.05
	test.That(t, cfg.streamLimitToleranceRatio(), test.ShouldEqual, 1.05)
	_, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldBeNil)

	cfg.StreamLimitToleranceRatio = 0.9
	_, _, err = cfg.Validate("")
	test.That(t, err, test.ShouldNotBeNil)
}

// armDir returns the absolute path to the arm/ directory containing test data.
func armDir() string {
	//nolint:dogsled
	_, file, _, _ := runtime.Caller(0)
	return filepath.Dir(file)
}

// moduleRoot points VIAM_MODULE_ROOT at the repository, where the shipped kinematic model files
// are, the way the server points it at the unpacked module.
func moduleRoot(t *testing.T) {
	t.Helper()
	t.Setenv("VIAM_MODULE_ROOT", filepath.Dir(armDir()))
}

func simpleModel(t *testing.T, m referenceframe.Model) *referenceframe.SimpleModel {
	t.Helper()
	sm, ok := m.(*referenceframe.SimpleModel)
	test.That(t, ok, test.ShouldBeTrue)
	return sm
}

// Every arm loads from its shipped file with collision geometry on every link, and the arms the
// viewer has GLB files for carry them as visuals.
func TestMakeModelFrame(t *testing.T) {
	logger := logging.NewTestLogger(t)
	moduleRoot(t)

	for _, tc := range []struct {
		model      string
		dof        int
		collisions int
		visuals    int
	}{
		{ModelName6DOF, 6, 7, 7},
		// the xArm7's URDF has no mesh for the flange and its SVA document no box, so one link is bare
		{ModelName7DOF, 7, 7, 0},
		{ModelNameLite, 6, 7, 7},
		{ModelName850, 6, 7, 0},
	} {
		t.Run(tc.model, func(t *testing.T) {
			m, err := MakeModelFrame("my-arm", tc.model, nil, nil, logger, 0, 0, 0, 0)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, m.Name(), test.ShouldEqual, "my-arm")
			test.That(t, len(m.DoF()), test.ShouldEqual, tc.dof)

			test.That(t, modelGeometries(t, m), test.ShouldHaveLength, tc.collisions)

			sm := simpleModel(t, m)
			visuals := 0
			for _, link := range sm.ModelConfig().Links {
				visuals += len(sm.VisualGeometries(link.ID))
			}
			test.That(t, visuals, test.ShouldEqual, tc.visuals)
		})
	}
}

// The 1305 wrist has its own collision mesh for the gripper mount, where the base xArm6 keeps
// the box its SVA document gives it.
func TestMakeModelFrameVariant(t *testing.T) {
	logger := logging.NewTestLogger(t)
	moduleRoot(t)

	mountGeometry := func(armTypeCode int) spatialmath.Geometry {
		m, err := MakeModelFrame("", ModelName6DOF, nil, nil, logger, armTypeCode, 0, 0, 0)
		test.That(t, err, test.ShouldBeNil)
		for _, g := range modelGeometries(t, m) {
			if g.Label() == m.Name()+":gripper_mount" {
				return g
			}
		}
		t.Fatal("no gripper_mount geometry")
		return nil
	}
	_, isMesh := mountGeometry(armTypeCode1305).(*spatialmath.Mesh)
	test.That(t, isMesh, test.ShouldBeTrue)
	_, isMesh = mountGeometry(0).(*spatialmath.Mesh)
	test.That(t, isMesh, test.ShouldBeFalse)
}

func TestMakeModelFrameMissingRoot(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Setenv("VIAM_MODULE_ROOT", "")
	_, err := MakeModelFrame("", ModelName6DOF, nil, nil, logger, 0, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "VIAM_MODULE_ROOT")

	t.Setenv("VIAM_MODULE_ROOT", "/nonexistent/path")
	_, err = MakeModelFrame("", ModelName6DOF, nil, nil, logger, 0, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
}

func TestMakeModelFrameUnknownModel(t *testing.T) {
	logger := logging.NewTestLogger(t)
	moduleRoot(t)

	_, err := MakeModelFrame("", "unknownModel", nil, nil, logger, 0, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no kinematics artifact for xarm model")
}

func TestMakeModelFrameBadJointsOutOfRange(t *testing.T) {
	logger := logging.NewTestLogger(t)
	moduleRoot(t)

	current := make([]referenceframe.Input, 6)

	_, err := MakeModelFrame("", ModelName6DOF, []int{99}, current, logger, 0, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "out of range")

	_, err = MakeModelFrame("", ModelName6DOF, []int{-1}, current, logger, 0, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
}

// The one degree of slack has to stay inside the joint, because a joint fails against its stop
// as often as anywhere else and the published bound would otherwise sit past it, where the
// motion service plans and the controller refuses. A joint that reports a position its own
// file calls impossible is locked at the nearest bound, since a bound past the stop would widen
// the hardware range and the model refuses that.
func TestLockedJointRangeDegs(t *testing.T) {
	elbow := referenceframe.JointConfig{ID: "elbow", Min: -225, Max: 10}

	for _, tc := range []struct {
		name           string
		atDegs         float64
		wantLo, wantHi float64
		wantOutside    bool
	}{
		{"on the upper stop", 10, 9, 10, false},
		{"on the lower stop", -225, -225, -224, false},
		{"well inside", -30, -31, -29, false},
		{"just past the upper stop", 10.5, 9.5, 10, false},
		{"past the upper stop", 30, 10, 10, true},
		{"past the lower stop", -300, -225, -225, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			lo, hi, outside := lockedJointRangeDegs(utils.DegToRad(tc.atDegs), elbow)
			test.That(t, lo, test.ShouldAlmostEqual, tc.wantLo, 1e-8)
			test.That(t, hi, test.ShouldAlmostEqual, tc.wantHi, 1e-8)
			test.That(t, outside, test.ShouldEqual, tc.wantOutside)
		})
	}
}

// Everything below reads what RDK actually sends, the typed model and the deprecated bytes,
// rather than the model we happen to be holding, because a limit that only exists in memory is
// the failure this is about.
func TestMakeModelFrameServedLimits(t *testing.T) {
	const speed, accel, hz = 45.0, 300.0, 120.0
	moduleRoot(t)

	// the xArm6 elbow lives in [-225, 10], so a lock there lands at [-31, -29]
	elbowAt30Below := make([]referenceframe.Input, 6)
	elbowAt30Below[2] = utils.DegToRad(-30)

	for _, tc := range []struct {
		name         string
		badJoints    []int
		current      []referenceframe.Input
		speed, accel float64
		wantSpeeds   bool
		wantElbow    *[2]float64
	}{
		{
			name:  "speeds alone reach every joint",
			speed: speed, accel: accel, wantSpeeds: true,
		},
		{
			// the arm must not claim bounds nobody chose, or the motion service plans timing
			// against numbers that came from us rather than from the hardware
			name: "no speeds means no advertised bounds",
		},
		{
			name: "a lock alone reaches the wire", badJoints: []int{2}, current: elbowAt30Below,
			wantElbow: &[2]float64{-31, -29},
		},
		{
			// both edits land in the same map entry, so a careless merge drops one of them
			name: "a lock and speeds survive each other", badJoints: []int{2}, current: elbowAt30Below,
			speed: speed, accel: accel, wantSpeeds: true, wantElbow: &[2]float64{-31, -29},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			logger := logging.NewTestLogger(t)
			m, err := MakeModelFrame("", ModelName6DOF, tc.badJoints, tc.current, logger, 0, tc.speed, tc.accel, hz)
			test.That(t, err, test.ShouldBeNil)

			resp := referenceframe.KinematicModelToProtobuf(m)
			typed := resp.GetModel()
			test.That(t, typed.GetProperties().GetOperatingFreqHz(), test.ShouldEqual, hz)
			test.That(t, typed.GetJoints(), test.ShouldHaveLength, 6)
			for _, joint := range typed.GetJoints() {
				if tc.wantSpeeds {
					test.That(t, joint.GetLimits().GetMaxVelocity(), test.ShouldAlmostEqual, tc.speed, 1e-9)
					test.That(t, joint.GetLimits().GetMaxAcceleration(), test.ShouldAlmostEqual, tc.accel, 1e-9)
				} else {
					test.That(t, joint.GetLimits().GetMaxVelocity(), test.ShouldEqual, 0)
					test.That(t, joint.GetLimits().GetMaxAcceleration(), test.ShouldEqual, 0)
				}
			}
			if tc.wantElbow != nil {
				elbow := typed.GetJoints()[2].GetLimits()
				test.That(t, elbow.GetMin(), test.ShouldAlmostEqual, tc.wantElbow[0], 1e-8)
				test.That(t, elbow.GetMax(), test.ShouldAlmostEqual, tc.wantElbow[1], 1e-8)
			}

			// a client older than the typed model reads the deprecated bytes
			legacy, err := referenceframe.UnmarshalModelJSON(resp.GetKinematicsData(), "") //nolint:staticcheck // served for old clients
			test.That(t, err, test.ShouldBeNil)
			vels, accs, ok := referenceframe.TrajectoryLimits(legacy.DoF())
			test.That(t, ok, test.ShouldEqual, tc.wantSpeeds)
			if tc.wantSpeeds {
				for i := range vels {
					test.That(t, vels[i], test.ShouldAlmostEqual, utils.DegToRad(tc.speed), 1e-8)
					test.That(t, accs[i], test.ShouldAlmostEqual, utils.DegToRad(tc.accel), 1e-8)
				}
			}
			if tc.wantElbow != nil {
				elbow := legacy.DoF()[2]
				test.That(t, utils.RadToDeg(elbow.Min), test.ShouldAlmostEqual, tc.wantElbow[0], 1e-8)
				test.That(t, utils.RadToDeg(elbow.Max), test.ShouldAlmostEqual, tc.wantElbow[1], 1e-8)
			}
		})
	}
}

// A joint reporting a position outside its range is locked at the nearest bound rather than
// refused, since the arm still has to come up for the other joints to be driven.
func TestMakeModelFrameLocksAnOutOfRangeJoint(t *testing.T) {
	logger := logging.NewTestLogger(t)
	moduleRoot(t)

	current := make([]referenceframe.Input, 6)
	current[2] = utils.DegToRad(30) // past the elbow's upper stop at 10

	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, logger, 0, 0, 0, 0)
	test.That(t, err, test.ShouldBeNil)
	elbow := m.DoF()[2]
	test.That(t, utils.RadToDeg(elbow.Min), test.ShouldAlmostEqual, 10, 1e-8)
	test.That(t, utils.RadToDeg(elbow.Max), test.ShouldAlmostEqual, 10, 1e-8)
}

func TestResolveArmKinematicsArtifact(t *testing.T) {
	cases := []struct {
		name        string
		model       string
		detected    detectedArm
		wantFile    string
		wantVariant string
		wantErr     bool
	}{
		{
			name:     "xArm6 base, no detection",
			model:    ModelName6DOF,
			detected: detectedArm{},
			wantFile: "xarm6",
		},
		{
			name:        "xArm6 with 1305 hardware variant",
			model:       ModelName6DOF,
			detected:    detectedArm{armTypeCode: 1305},
			wantFile:    "xarm6_1305",
			wantVariant: "1305",
		},
		{
			name:     "xArm6 with unknown armTypeCode falls back to base",
			model:    ModelName6DOF,
			detected: detectedArm{armTypeCode: 9999},
			wantFile: "xarm6",
		},
		{
			name:     "xArm850 base",
			model:    ModelName850,
			detected: detectedArm{},
			wantFile: "uf850",
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
			test.That(t, got.file, test.ShouldEqual, tc.wantFile)
			test.That(t, got.variant, test.ShouldEqual, tc.wantVariant)
		})
	}
}

// Every artifact the tables name ships as a file.
func TestKinematicsArtifactsOnDisk(t *testing.T) {
	moduleRoot(t)
	var artifacts []kinematicsArtifact
	for _, a := range armKinematicsBase {
		artifacts = append(artifacts, a)
	}
	for _, a := range armKinematicsVariants {
		artifacts = append(artifacts, a)
	}
	for _, a := range gripperKinematicsBase {
		artifacts = append(artifacts, a)
	}
	for _, a := range artifacts {
		path, err := kinematicsPath(a.file)
		test.That(t, err, test.ShouldBeNil)
		_, err = os.Stat(path)
		test.That(t, err, test.ShouldBeNil)
	}
}

// Get3DModels serves the GLB each link carries, keyed by link, the way the viewer expects.
func TestGet3DModels(t *testing.T) {
	logger := logging.NewTestLogger(t)
	moduleRoot(t)

	for _, tc := range []struct {
		model string
		links []string
	}{
		{ModelName6DOF, []string{"base", "base_top", "upper_arm", "upper_forearm", "lower_forearm", "wrist_link", "gripper_mount"}},
		{ModelName7DOF, nil},
	} {
		t.Run(tc.model, func(t *testing.T) {
			m, err := MakeModelFrame("", tc.model, nil, nil, logger, 0, 0, 0, 0)
			test.That(t, err, test.ShouldBeNil)
			x := &xArm{model: m}
			models, err := x.Get3DModels(context.Background(), nil)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, models, test.ShouldHaveLength, len(tc.links))
			for _, link := range tc.links {
				test.That(t, models[link].GetContentType(), test.ShouldEqual, "model/gltf-binary")
				test.That(t, models[link].GetMesh(), test.ShouldNotBeEmpty)
			}
		})
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
