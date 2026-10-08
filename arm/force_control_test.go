package arm

import (
	"encoding/binary"
	"math"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

// leF32 reads a little-endian float32 out of a payload, the way the controller
// will.
func leF32(t *testing.T, b []byte, at int) float64 {
	t.Helper()
	test.That(t, len(b) >= at+4, test.ShouldBeTrue)
	return float64(math.Float32frombits(binary.LittleEndian.Uint32(b[at : at+4])))
}

// putLEF32 writes one, for building synthetic controller responses.
func putLEF32(b []byte, at int, v float64) {
	binary.LittleEndian.PutUint32(b[at:at+4], math.Float32bits(float32(v)))
}

func TestForceCtrlConfigParams(t *testing.T) {
	// The case this whole feature exists for: Z force-controlled at 5N, XY and
	// all rotations left under position control.
	axes := forceAxes{false, false, true, false, false, false}
	ref := forceVec{0, 0, -5, 0, 0, 0}

	got := forceCtrlConfigParams(forceFrameBase, axes, ref)

	// Length is load-bearing: the controller reads a fixed 55 bytes, so a short
	// payload is not rejected, it is misparsed.
	test.That(t, len(got), test.ShouldEqual, forceCtrlConfigLen)

	// coord u8 @0, c_axis 6*u8 @1
	test.That(t, got[0], test.ShouldEqual, forceFrameBase)
	test.That(t, got[1:7], test.ShouldResemble, []byte{0, 0, 1, 0, 0, 0})

	// f_ref 6*float32 @7
	for i, want := range ref {
		test.That(t, leF32(t, got, 7+i*4), test.ShouldAlmostEqual, want, 1e-6)
	}

	// limits 6*float32 @31, reserved, must be zeros
	for i := range 6 {
		test.That(t, leF32(t, got, 31+i*4), test.ShouldEqual, 0.0)
	}

	// Tool frame is the only other legal value.
	tool := forceCtrlConfigParams(forceFrameTool, axes, ref)
	test.That(t, tool[0], test.ShouldEqual, forceFrameTool)
}

func TestForceCtrlConfigParamsAllAxes(t *testing.T) {
	// Every axis compliant, distinct targets, so a transposed or off-by-one
	// offset shows up as a mismatched value rather than a coincidence.
	axes := forceAxes{true, true, true, true, true, true}
	ref := forceVec{1, -2, 3, -0.4, 0.5, -0.6}

	got := forceCtrlConfigParams(forceFrameTool, axes, ref)
	test.That(t, len(got), test.ShouldEqual, forceCtrlConfigLen)
	test.That(t, got[1:7], test.ShouldResemble, []byte{1, 1, 1, 1, 1, 1})
	for i, want := range ref {
		test.That(t, leF32(t, got, 7+i*4), test.ShouldAlmostEqual, want, 1e-6)
	}
}

func TestForceCtrlPIDParams(t *testing.T) {
	// Distinct values per block so a swapped block is caught.
	got := forceCtrlPIDParams(
		uniformVec(0.005), uniformVec(0.00006), uniformVec(0.01), uniformVec(100))

	test.That(t, len(got), test.ShouldEqual, forceCtrlPIDLen)

	for i := range 6 {
		test.That(t, leF32(t, got, 0+i*4), test.ShouldAlmostEqual, 0.005, 1e-9)
		test.That(t, leF32(t, got, 24+i*4), test.ShouldAlmostEqual, 0.00006, 1e-9)
		test.That(t, leF32(t, got, 48+i*4), test.ShouldAlmostEqual, 0.01, 1e-9)
		test.That(t, leF32(t, got, 72+i*4), test.ShouldAlmostEqual, 100.0, 1e-6)
	}
}

func TestForceCtrlPIDClamping(t *testing.T) {
	// Out of range clamps rather than erroring: these are tuning knobs, and the
	// controller would otherwise take a nonsense gain at face value.
	got := forceCtrlPIDParams(
		uniformVec(99),   // kp, way over
		uniformVec(-1),   // ki, negative
		uniformVec(99),   // kd, way over
		uniformVec(9999), // xe_limit, way over
	)

	test.That(t, leF32(t, got, 0), test.ShouldAlmostEqual, forceKpMax, 1e-9)
	test.That(t, leF32(t, got, 24), test.ShouldEqual, 0.0)
	test.That(t, leF32(t, got, 48), test.ShouldAlmostEqual, forceKdMax, 1e-9)
	test.That(t, leF32(t, got, 72), test.ShouldAlmostEqual, forceXeLimitMax, 1e-6)

	// In-range values pass through untouched.
	ok := forceCtrlPIDParams(
		uniformVec(0.004), uniformVec(0.0001), uniformVec(0.02), uniformVec(50))
	test.That(t, leF32(t, ok, 0), test.ShouldAlmostEqual, 0.004, 1e-9)
	test.That(t, leF32(t, ok, 24), test.ShouldAlmostEqual, 0.0001, 1e-9)
	test.That(t, leF32(t, ok, 48), test.ShouldAlmostEqual, 0.02, 1e-9)
	test.That(t, leF32(t, ok, 72), test.ShouldAlmostEqual, 50.0, 1e-6)
}

func TestParseFTSensorConfig(t *testing.T) {
	// Build a controller response: one leading state byte, then the 280-byte
	// blob. Offsets here are the SDK's; the parser adds the +1 for the state
	// byte, so writing at the raw offset + 1 is the round trip under test.
	params := make([]byte, 1+ftSensorConfigLen)
	params[0] = 0x00 // state

	params[ftCfgMode] = ftModeForce
	params[ftCfgIsStarted] = 1
	params[ftCfgType] = 4
	params[ftCfgID] = 8
	binary.BigEndian.PutUint16(params[ftCfgFreq:ftCfgFreq+2], 200) // integers are BE
	params[ftCfgFCoord] = forceFrameTool

	wantAxes := forceAxes{false, false, true, false, false, false}
	for i, on := range wantAxes {
		if on {
			params[ftCfgFAxes+i] = 1
		}
	}

	wantRef := forceVec{0, 0, -5, 0, 0, 0}
	for i, v := range wantRef {
		putLEF32(params, ftCfgFRef+i*4, v)
	}
	for i := range 6 {
		putLEF32(params, ftCfgKp+i*4, 0.005)
		putLEF32(params, ftCfgKi+i*4, 0.00006)
		putLEF32(params, ftCfgKd+i*4, 0.01)
		putLEF32(params, ftCfgXeLimit+i*4, 100)
	}

	cfg, err := parseFTSensorConfig(params)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.Mode, test.ShouldEqual, ftModeForce)
	test.That(t, cfg.IsStarted, test.ShouldBeTrue)
	test.That(t, cfg.Type, test.ShouldEqual, byte(4))
	test.That(t, cfg.ID, test.ShouldEqual, byte(8))
	test.That(t, cfg.Freq, test.ShouldEqual, uint16(200))
	test.That(t, cfg.Coord, test.ShouldEqual, forceFrameTool)
	test.That(t, cfg.Axes, test.ShouldResemble, wantAxes)
	for i := range 6 {
		test.That(t, cfg.Ref[i], test.ShouldAlmostEqual, wantRef[i], 1e-6)
		test.That(t, cfg.Kp[i], test.ShouldAlmostEqual, 0.005, 1e-9)
		test.That(t, cfg.XeLimit[i], test.ShouldAlmostEqual, 100.0, 1e-6)
	}

	// A short response must error rather than index out of range.
	_, err = parseFTSensorConfig(make([]byte, 32))
	test.That(t, err, test.ShouldNotBeNil)
}

func TestParseFTSensorConfigAsMap(t *testing.T) {
	params := make([]byte, 1+ftSensorConfigLen)
	params[ftCfgMode] = ftModeOff
	params[ftCfgFCoord] = forceFrameBase

	cfg, err := parseFTSensorConfig(params)
	test.That(t, err, test.ShouldBeNil)

	m := cfg.asMap()
	test.That(t, m["mode"], test.ShouldEqual, 0)
	test.That(t, m["mode_name"], test.ShouldEqual, "off")
	test.That(t, m["is_started"], test.ShouldEqual, false)
	test.That(t, m["frame"], test.ShouldEqual, "base")
	test.That(t, len(m["axes"].([]any)), test.ShouldEqual, 6)
	test.That(t, len(m["ref"].([]any)), test.ShouldEqual, 6)
}

func TestParseForceControlRequest(t *testing.T) {
	zOnly := forceAxes{false, false, true, false, false, false}

	t.Run("shorthand single axis and force", func(t *testing.T) {
		req, err := parseForceControlRequest(map[string]any{
			"axes": "z", "forces": -5.0,
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req.axes, test.ShouldResemble, zOnly)
		test.That(t, req.ref[2], test.ShouldAlmostEqual, -5.0, 1e-9)
		// Defaults, and base frame when unspecified.
		test.That(t, req.coord, test.ShouldEqual, forceFrameBase)
		test.That(t, req.kp[0], test.ShouldAlmostEqual, defaultForceKp, 1e-9)
		test.That(t, req.xeLimit[0], test.ShouldAlmostEqual, defaultForceXeLimit, 1e-9)
	})

	t.Run("named lists and maps", func(t *testing.T) {
		req, err := parseForceControlRequest(map[string]any{
			"axes":   []any{"z"},
			"forces": map[string]any{"z": -3.0},
			"frame":  "tool",
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req.axes, test.ShouldResemble, zOnly)
		test.That(t, req.ref[2], test.ShouldAlmostEqual, -3.0, 1e-9)
		test.That(t, req.coord, test.ShouldEqual, forceFrameTool)
	})

	t.Run("explicit six vectors", func(t *testing.T) {
		req, err := parseForceControlRequest(map[string]any{
			"axes":   []any{false, false, true, false, false, false},
			"forces": []any{0.0, 0.0, -5.0, 0.0, 0.0, 0.0},
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req.axes, test.ShouldResemble, zOnly)
		test.That(t, req.ref[2], test.ShouldAlmostEqual, -5.0, 1e-9)
	})

	t.Run("gains override", func(t *testing.T) {
		req, err := parseForceControlRequest(map[string]any{
			"axes": "z", "forces": -5.0,
			"kp": 0.01, "ki": 0.0001, "kd": 0.02, "max_velocity": 40.0,
		})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req.kp[0], test.ShouldAlmostEqual, 0.01, 1e-9)
		test.That(t, req.ki[0], test.ShouldAlmostEqual, 0.0001, 1e-9)
		test.That(t, req.kd[0], test.ShouldAlmostEqual, 0.02, 1e-9)
		test.That(t, req.xeLimit[0], test.ShouldAlmostEqual, 40.0, 1e-9)
	})

	t.Run("integers survive the JSON round trip", func(t *testing.T) {
		// protobuf Struct gives float64, but a Go caller may pass int.
		req, err := parseForceControlRequest(map[string]any{"axes": "z", "forces": -5})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, req.ref[2], test.ShouldAlmostEqual, -5.0, 1e-9)
	})

	for _, tc := range []struct {
		name string
		in   any
	}{
		{"not a map", "z"},
		{"no axes", map[string]any{"forces": -5.0}},
		{"no forces", map[string]any{"axes": "z"}},
		{"unknown axis", map[string]any{"axes": "q", "forces": -5.0}},
		{"unknown axis in forces map", map[string]any{"axes": "z", "forces": map[string]any{"q": -5.0}}},
		{"bad frame", map[string]any{"axes": "z", "forces": -5.0, "frame": "world"}},
		{"frame not a string", map[string]any{"axes": "z", "forces": -5.0, "frame": 1.0}},
		{"wrong length force list", map[string]any{"axes": "z", "forces": []any{0.0, -5.0}}},
		{"gain not a number", map[string]any{"axes": "z", "forces": -5.0, "kp": "fast"}},
		// A bare number cannot say which of two axes it belongs to.
		{"ambiguous bare force", map[string]any{"axes": []any{"y", "z"}, "forces": -5.0}},
		// Compliant with a zero target reads as the arm ignoring the caller.
		{"compliant axis with zero force", map[string]any{"axes": "z", "forces": []any{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},
	} {
		t.Run("rejects "+tc.name, func(t *testing.T) {
			_, err := parseForceControlRequest(tc.in)
			test.That(t, err, test.ShouldNotBeNil)
		})
	}
}

func TestForceAxesAny(t *testing.T) {
	test.That(t, (forceAxes{}).any(), test.ShouldBeFalse)
	test.That(t, (forceAxes{false, false, true, false, false, false}).any(), test.ShouldBeTrue)
}

// Force control only holds in motion mode 0, so an ordinary move must not be
// allowed to flip the arm back to servo mode while it is armed.
func TestMoveOptionsPinsMode0UnderForceControl(t *testing.T) {
	x := &xArm{speed: 1, acceleration: 1, moveHZ: defaultMoveHz, logger: logging.NewTestLogger(t)}

	mo := x.moveOptions(nil, nil)
	test.That(t, mo.mode0, test.ShouldBeFalse)
	test.That(t, mo.direct, test.ShouldBeFalse)

	x.forceControlActive.Store(true)
	mo = x.moveOptions(nil, nil)
	test.That(t, mo.mode0, test.ShouldBeTrue)
	// An explicit request for the servo path is overridden, not honoured.
	test.That(t, x.moveOptions(nil, map[string]any{"direct": false}).mode0, test.ShouldBeTrue)

	x.forceControlActive.Store(false)
	test.That(t, x.moveOptions(nil, nil).mode0, test.ShouldBeFalse)

	// `direct` still implies mode0 on its own, for callers that asked for it.
	test.That(t, x.moveOptions(nil, map[string]any{"direct": true}).mode0, test.ShouldBeTrue)
}

// Regression: pinning the motion mode must not also pin `direct`.
//
// internalMoveThroughJointPositions rejects `direct` with more than one
// waypoint, because direct skips interpolation and would have the arm jumping
// between distant configurations at full speed. A drawing stroke is dozens of
// waypoints, so conflating "run in mode 0" with "direct" would make every
// force-controlled path move fail with "direct only work with 1 position".
func TestForceControlDoesNotForceDirect(t *testing.T) {
	x := &xArm{speed: 1, acceleration: 1, moveHZ: defaultMoveHz, logger: logging.NewTestLogger(t)}
	x.forceControlActive.Store(true)

	mo := x.moveOptions(nil, nil)
	test.That(t, mo.mode0, test.ShouldBeTrue)
	test.That(t, mo.direct, test.ShouldBeFalse)
	// Interpolation must survive too, or the waypoints go out raw.
	test.That(t, mo.interpolate, test.ShouldBeTrue)
}

// Motion mode 0 must never be handed a densified stream. Its opcode is a
// planned point-to-point move, so a hundred setpoints a second becomes a
// hundred queued plans a second and the arm crawls. Force control pins the arm
// to mode 0, so this is what made the drawing phase -- and only the drawing
// phase -- slow.
func TestChooseStepSource(t *testing.T) {
	for _, tc := range []struct {
		name       string
		hasTrajGen bool
		mo         moveOptions
		want       stepSource
	}{
		{"servo mode with a trajectory generator densifies", true,
			moveOptions{interpolate: true}, stepSourceTrajGen},
		{"servo mode without one interpolates", false,
			moveOptions{interpolate: true}, stepSourceInterpolate},
		{"servo mode with interpolation off sends raw", false,
			moveOptions{interpolate: false}, stepSourceRaw},

		// The fix.
		{"mode 0 sends raw even with a trajectory generator", true,
			moveOptions{mode0: true, interpolate: true}, stepSourceRaw},
		{"mode 0 sends raw without one", false,
			moveOptions{mode0: true, interpolate: true}, stepSourceRaw},

		// `direct` implies mode0, and previously still went through trajex
		// because that branch was checked first and ignored direct.
		{"direct sends raw with a trajectory generator", true,
			moveOptions{direct: true, mode0: true, interpolate: true}, stepSourceRaw},
	} {
		t.Run(tc.name, func(t *testing.T) {
			test.That(t, chooseStepSource(tc.hasTrajGen, tc.mo), test.ShouldEqual, tc.want)
		})
	}
}

// A force-controlled stroke is many waypoints in mode 0, so the two must
// compose: pinning the mode must not densify, and must not trip the
// single-waypoint restriction that `direct` carries.
func TestForceControlStrokeSendsRawWaypoints(t *testing.T) {
	x := &xArm{speed: 1, acceleration: 1, moveHZ: defaultMoveHz, logger: logging.NewTestLogger(t)}
	x.forceControlActive.Store(true)

	mo := x.moveOptions(nil, nil)
	test.That(t, mo.mode0, test.ShouldBeTrue)
	test.That(t, mo.direct, test.ShouldBeFalse)
	test.That(t, chooseStepSource(true, mo), test.ShouldEqual, stepSourceRaw)
}

func TestStepSourceString(t *testing.T) {
	test.That(t, stepSourceRaw.String(), test.ShouldEqual, "raw")
	test.That(t, stepSourceTrajGen.String(), test.ShouldEqual, "trajgen")
	test.That(t, stepSourceInterpolate.String(), test.ShouldEqual, "interpolate")
}

// Blending is opt-in and only applies in mode 0. It swaps MOVE_JOINT for
// MOVE_JOINTB, whose trailing float is a blend radius rather than the unused
// motion-time field -- which is what stops the arm decelerating to zero at
// every waypoint.
func TestBlendRadiusOption(t *testing.T) {
	x := &xArm{speed: 1, acceleration: 1, moveHZ: defaultMoveHz, logger: logging.NewTestLogger(t)}

	test.That(t, x.moveOptions(nil, nil).blendRadius, test.ShouldBeLessThan, 0)

	mo := x.moveOptions(nil, map[string]any{"blend_radius": 1.5})
	test.That(t, mo.blendRadius, test.ShouldAlmostEqual, 1.5, 1e-9)

	// Zero is a legal radius and must not read as "disabled".
	test.That(t, x.moveOptions(nil, map[string]any{"blend_radius": 0.0}).blendRadius,
		test.ShouldEqual, 0.0)
}

// The blended opcode must carry the radius in the slot MOVE_JOINT leaves empty,
// and the payload length must not change.
func TestSendJointStepBlendedPayload(t *testing.T) {
	radius := 1.5
	plain := jointStepParams([]float64{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, 6, 1.0, 2.0, false, 0)
	blend := jointStepParams([]float64{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, 6, 1.0, 2.0, true, radius)

	test.That(t, len(blend), test.ShouldEqual, len(plain))
	// 7 joints + speed + accel = 9 floats, then the trailing one.
	test.That(t, len(blend), test.ShouldEqual, 10*4)
	test.That(t, leF32(t, plain, 9*4), test.ShouldEqual, 0.0)
	test.That(t, leF32(t, blend, 9*4), test.ShouldAlmostEqual, radius, 1e-6)
	// Everything before the trailing float is identical.
	test.That(t, blend[:9*4], test.ShouldResemble, plain[:9*4])
}
