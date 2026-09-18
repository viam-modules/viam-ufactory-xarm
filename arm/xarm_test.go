package arm

import (
	"math"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	commonpb "go.viam.com/api/common/v1"
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

// On the URDF path a lock cannot reach the server, since the bytes RDK sends are the URDF off
// disk. It still has to shape the model we return, so this module's own planning refuses to move
// the joint. Losing that quietly is the failure this pins down.
func TestMakeModelFrameBadJointsOnURDFReachTheWire(t *testing.T) {
	logger := logging.NewTestLogger(t)

	repoRoot := filepath.Dir(armDir())
	t.Setenv("VIAM_MODULE_ROOT", repoRoot)

	current := make([]referenceframe.Input, 6)
	current[2] = utils.DegToRad(-30) // the xArm6 elbow lives in [-225, 10]

	m, err := MakeModelFrame("", ModelName6DOF, []int{2}, current, true, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldBeNil)

	locked := m.DoF()[2]
	test.That(t, utils.RadToDeg(locked.Min), test.ShouldAlmostEqual, -31.0, 1e-8)
	test.That(t, utils.RadToDeg(locked.Max), test.ShouldAlmostEqual, -29.0, 1e-8)

	// The file we parsed is still the URDF, but the lock travels on the typed model as a user
	// limit, with the hardware range left as the URDF declared it.
	test.That(t, m.ModelConfig().OriginalFile.Extension, test.ShouldEqual, "urdf")
	elbow := referenceframe.KinematicModelToProtobuf(m).GetModel().GetJoints()[2]
	test.That(t, elbow.GetUserLimits().GetMin(), test.ShouldAlmostEqual, -31.0, 1e-8)
	test.That(t, elbow.GetUserLimits().GetMax(), test.ShouldAlmostEqual, -29.0, 1e-8)
	// the URDF's own range, untouched by the lock, still contains it
	test.That(t, elbow.GetHardwareLimits().GetMin(), test.ShouldBeLessThan, -31.0)
	test.That(t, elbow.GetHardwareLimits().GetMax(), test.ShouldBeGreaterThan, -29.0)
}

func TestMakeModelFrameBadJointsOutOfRange(t *testing.T) {
	logger := logging.NewTestLogger(t)

	current := make([]referenceframe.Input, 6)

	_, err := MakeModelFrame("", ModelName6DOF, []int{99}, current, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "out of range")

	_, err = MakeModelFrame("", ModelName6DOF, []int{-1}, current, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldNotBeNil)
}

// A joint fails against its stop as often as anywhere else, and the slack would then publish a
// bound past the stop that the motion service would plan to and the controller would refuse.
// The one degree of slack has to stay inside the joint, because a joint fails against its stop as
// often as anywhere else and the published bound would otherwise sit past it, where the motion
// service plans and the controller refuses. Except when the joint reports a position its own
// document calls impossible: that is still where the arm is, and moving the window would make
// every pose computed from this joint wrong.
func TestLockedJointRangeDegs(t *testing.T) {
	elbow := referenceframe.JointConfig{ID: "elbow", Min: -225, Max: 10}

	for _, tc := range []struct {
		name           string
		atDegs         float64
		wantLo, wantHi float64
	}{
		{"on the upper stop", 10, 9, 10},
		{"on the lower stop", -225, -225, -224},
		{"well inside", -30, -31, -29},
		{"past the upper stop", 30, 29, 31},
	} {
		t.Run(tc.name, func(t *testing.T) {
			lo, hi := lockedJointRangeDegs(utils.DegToRad(tc.atDegs), elbow)
			test.That(t, lo, test.ShouldAlmostEqual, tc.wantLo, 1e-8)
			test.That(t, hi, test.ShouldAlmostEqual, tc.wantHi, 1e-8)
		})
	}
}

func TestUseURDFsDefaultsFalse(t *testing.T) {
	cfg := &Config{}
	test.That(t, cfg.UseURDFs, test.ShouldBeFalse)
}

// Everything below reads the bytes RDK actually sends rather than the model we happen to be
// holding, because a limit that only exists in memory is the failure this whole change is about.
func TestMakeModelFrameServedLimits(t *testing.T) {
	const speed, accel = 45.0, 300.0

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
			m, err := MakeModelFrame("", ModelName6DOF, tc.badJoints, tc.current, false, nil, logger, 0, tc.speed, tc.accel)
			test.That(t, err, test.ShouldBeNil)

			// what a client sees is the typed model, read back the way the framesystem reads it
			resp := referenceframe.KinematicModelToProtobuf(m)
			served, err := referenceframe.KinematicModelFromProtobuf("", resp)
			test.That(t, err, test.ShouldBeNil)

			vels, accs, ok := referenceframe.TrajectoryLimits(served.DoF())
			test.That(t, ok, test.ShouldEqual, tc.wantSpeeds)
			if tc.wantSpeeds {
				test.That(t, vels, test.ShouldHaveLength, 6)
				for i := range vels {
					test.That(t, vels[i], test.ShouldAlmostEqual, utils.DegToRad(tc.speed), 1e-8)
					test.That(t, accs[i], test.ShouldAlmostEqual, utils.DegToRad(tc.accel), 1e-8)
				}
			}

			if tc.wantElbow != nil {
				elbow := served.DoF()[2]
				test.That(t, utils.RadToDeg(elbow.Min), test.ShouldAlmostEqual, tc.wantElbow[0], 1e-8)
				test.That(t, utils.RadToDeg(elbow.Max), test.ShouldAlmostEqual, tc.wantElbow[1], 1e-8)
				// the lock narrows the user layer only; the hardware range is still what the file said
				hw := resp.GetModel().GetJoints()[2].GetHardwareLimits()
				test.That(t, hw.GetMin(), test.ShouldAlmostEqual, -225.0, 1e-8)
				test.That(t, hw.GetMax(), test.ShouldAlmostEqual, 10.0, 1e-8)
			}
		})
	}
}

func TestAttachVisualsAndProperties(t *testing.T) {
	logger := logging.NewTestLogger(t)
	t.Setenv("VIAM_MODULE_ROOT", filepath.Dir(armDir()))

	m, err := MakeModelFrame("", ModelName6DOF, nil, nil, false, nil, logger, 0, 0, 0)
	test.That(t, err, test.ShouldBeNil)
	conf := &Config{MoveHZ: 120}
	test.That(t, attachVisualsAndProperties(m, conf, ModelName6DOF, logger), test.ShouldBeNil)

	pb := referenceframe.KinematicModelToProtobuf(m).GetModel()
	test.That(t, pb.GetProperties().GetTrajectorySamplingFreqHz(), test.ShouldEqual, 120.0)

	visuals := 0
	for _, link := range pb.GetLinks() {
		for _, g := range link.GetVisual() {
			visuals++
			test.That(t, g.GetMesh().GetContentType(), test.ShouldEqual, "glb")
			test.That(t, g.GetMesh().GetSourcePath(), test.ShouldEqual, "3d_models/"+ModelName6DOF+"/"+link.GetId()+".glb")
			test.That(t, len(g.GetMesh().GetMesh()), test.ShouldBeGreaterThan, 0)
		}
	}
	test.That(t, visuals, test.ShouldEqual, len(armTo3DModelParts[ModelName6DOF]))

	// and a caller who does not want the visuals does not pay for them
	slim := referenceframe.KinematicModelToProtobufForRequest(m, &commonpb.GetKinematicsRequest{ExcludeVisualMeshes: true})
	for _, link := range slim.GetModel().GetLinks() {
		for _, g := range link.GetVisual() {
			test.That(t, len(g.GetMesh().GetMesh()), test.ShouldEqual, 0)
			test.That(t, g.GetMesh().GetSourcePath(), test.ShouldNotBeEmpty)
		}
	}
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

// With a module root, the 1305 variant loads from the shipped SVA v2 file: collision meshes and
// visuals arrive by path, the joints carry the SVA names, and the configured speed still lands
// as a user limit on each of them.
func TestMakeModelFramePrefersSVAv2(t *testing.T) {
	logger := logging.NewTestLogger(t)
	t.Setenv("VIAM_MODULE_ROOT", filepath.Dir(armDir()))

	m, err := MakeModelFrame("", ModelName6DOF, nil, nil, false, nil, logger, 1305, 45, 300)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(m.DoF()), test.ShouldEqual, 6)

	pb := referenceframe.KinematicModelToProtobuf(m).GetModel()
	names := map[string]bool{}
	meshLinks, visualLinks := 0, 0
	for _, l := range pb.GetLinks() {
		names[l.GetId()] = true
		if len(l.GetCollision()) > 0 && l.GetCollision()[0].GetMesh() != nil {
			meshLinks++
			test.That(t, len(l.GetCollision()[0].GetMesh().GetMesh()), test.ShouldBeGreaterThan, 0)
			test.That(t, l.GetCollision()[0].GetMesh().GetContentType(), test.ShouldEqual, "ply")
		}
		if len(l.GetVisual()) > 0 {
			visualLinks++
			test.That(t, len(l.GetVisual()[0].GetMesh().GetMesh()), test.ShouldBeGreaterThan, 0)
		}
	}
	test.That(t, meshLinks, test.ShouldEqual, 7)
	test.That(t, visualLinks, test.ShouldEqual, 7)
	test.That(t, names["gripper_mount"], test.ShouldBeTrue)

	for _, j := range pb.GetJoints() {
		test.That(t, j.GetUserLimits().GetMaxVelocity(), test.ShouldEqual, 45.0)
		test.That(t, j.GetUserLimits().GetMaxAcceleration(), test.ShouldEqual, 300.0)
	}
	test.That(t, pb.GetJoints()[0].GetId(), test.ShouldEqual, "waist")
}
