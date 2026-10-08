package arm

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"math"

	rutils "go.viam.com/rdk/utils"
)

// Hybrid position/force control through the UFactory 6-axis F/T sensor.
//
// The controller implements the hybrid controller itself; this file is only the
// wire encoding plus the arming sequence. The primitive is a per-axis compliance
// selection vector: axes flagged compliant are driven to a target force, the
// rest stay position-controlled. So {false,false,true,false,false,false} with a
// -5N Z target is "hold 5N down while XY tracks whatever motion is commanded".
//
// Layouts are taken from the upstream C++ SDK
// (src/xarm/core/instruction/uxbus_cmd.cc, config in uxbus_cmd_config.h). Floats
// are little-endian on the wire, matching nfp32_to_hex and the rest of this
// driver; integers in the command protocol are single bytes here so endianness
// does not arise.
//
// Requires controller firmware >= 2.3.0. The SDK header says 1.8.3, but moving
// while force control is active raised C31 ("abnormal current") below 2.3.0 --
// see xArm-Python-SDK issue #108 -- and moving while force-controlled is the
// entire point.

const (
	// ftModeOff disables compliance and returns the arm to pure position control.
	ftModeOff byte = 0
	// ftModeAdmittance is a spring-damper with no force setpoint (hand-guiding).
	// Not used for force control, but named so the numbering is self-describing
	// and so a controller reporting mode 1 can be explained rather than guessed at.
	ftModeAdmittance byte = 1
	// ftModeForce is closed-loop force control with a target and a PID loop.
	ftModeForce byte = 2
)

// Task frames for the compliance axes, and their DoCommand names.
const (
	forceFrameBase byte = 0
	forceFrameTool byte = 1

	forceFrameBaseName = "base"
	forceFrameToolName = "tool"
)

// Wire sizes, asserted in tests so a layout mistake fails loudly rather than
// being silently truncated by the controller.
const (
	forceCtrlConfigLen = 55  // coord + 6 axis flags + 6 f_ref + 6 limits
	forceCtrlPIDLen    = 96  // 4 gain vectors of 6 float32
	ftSensorConfigLen  = 280 // the controller's whole force-control state
)

// Gain ranges, from the SDK's 8003-force_control example and the Python client's
// own validation. Out-of-range values are clamped rather than rejected: these are
// tuning knobs, and refusing a move because a gain was 0.06 helps nobody.
const (
	forceKpMax      = 0.05
	forceKiMax      = 0.0005
	forceKdMax      = 0.05
	forceXeLimitMax = 200.0 // max correction velocity, mm/s

	defaultForceKp      = 0.005
	defaultForceKi      = 0.00006
	defaultForceKd      = 0.0
	defaultForceXeLimit = 100.0
)

// forceAxes selects which Cartesian axes are force-controlled. Index order is
// x, y, z, rx, ry, rz.
type forceAxes [6]bool

// forceVec is a per-axis quantity in the same index order: forces in newtons for
// the first three, torques in newton-metres for the last three.
type forceVec [6]float64

func (a forceAxes) any() bool {
	for _, v := range a {
		if v {
			return true
		}
	}
	return false
}

// putFloat32LE appends v to buf as a little-endian float32.
func putFloat32LE(buf []byte, v float64) []byte {
	b := make([]byte, 4)
	binary.LittleEndian.PutUint32(b, math.Float32bits(float32(v)))
	return append(buf, b...)
}

// forceCtrlConfigParams encodes the ForceCtrlConfig (0xD1) payload.
//
// Layout, 55 bytes: coord u8 @0, c_axis 6*u8 @1, f_ref 6*float32 @7,
// limits 6*float32 @31. `limits` is documented as a per-axis speed cap but the
// SDK examples pass zeros with the comment "limits are reserved"; the effective
// cap is xeLimit in the PID payload.
func forceCtrlConfigParams(coord byte, axes forceAxes, ref forceVec) []byte {
	params := make([]byte, 0, forceCtrlConfigLen)
	params = append(params, coord)
	for _, on := range axes {
		var b byte
		if on {
			b = 1
		}
		params = append(params, b)
	}
	for _, v := range ref {
		params = putFloat32LE(params, v)
	}
	for range ref {
		params = putFloat32LE(params, 0) // limits: reserved
	}
	return params
}

// forceCtrlPIDParams encodes the ForceCtrlPID (0xD0) payload.
//
// Layout, 96 bytes: kp @0, ki @24, kd @48, xe_limit @72, each 6 little-endian
// float32. Values are clamped to the documented ranges.
func forceCtrlPIDParams(kp, ki, kd, xeLimit forceVec) []byte {
	params := make([]byte, 0, forceCtrlPIDLen)
	for _, spec := range []struct {
		vals forceVec
		max  float64
	}{{kp, forceKpMax}, {ki, forceKiMax}, {kd, forceKdMax}, {xeLimit, forceXeLimitMax}} {
		for _, v := range spec.vals {
			params = putFloat32LE(params, clampForceGain(v, spec.max))
		}
	}
	return params
}

// clampForceGain holds v within [0, max]. Negative gains would drive the loop
// away from the target, so they clamp to zero rather than passing through.
func clampForceGain(v, maxVal float64) float64 {
	if v < 0 {
		return 0
	}
	if v > maxVal {
		return maxVal
	}
	return v
}

// uniformVec fills all six axes with the same value, which is how the SDK
// examples set gains.
func uniformVec(v float64) forceVec {
	var out forceVec
	for i := range out {
		out[i] = v
	}
	return out
}

// ftSensorConfig is the controller's force-control state, as reported by
// FTSensorGetConfig (0xD4). Only the fields that matter for diagnosing a force
// control session are decoded; the blob also carries the admittance parameters
// and the sensor's stored zero, which nothing here uses.
type ftSensorConfig struct {
	Mode      byte      // 0 off, 1 admittance, 2 force control
	IsStarted bool      // whether the mode is actually running
	Type      byte      //
	ID        byte      //
	Freq      uint16    // sensor sampling rate, NOT the 200Hz control rate
	Coord     byte      // force-control task frame: 0 base, 1 tool
	Axes      forceAxes // which axes are force-controlled
	Ref       forceVec  // target force/torque per axis
	Kp        forceVec
	Ki        forceVec
	Kd        forceVec
	XeLimit   forceVec
}

// Byte offsets within the 0xD4 response payload.
//
// These are the SDK's offsets plus one. The SDK's _recv_modbus_response strips
// the leading state byte before handing the payload to its decoder; this
// driver's responseInLock does not, which is the same reason parseFTSensorData
// reads floats at i*4+1. Getting this wrong shifts every field by a byte and
// yields plausible-looking garbage, so it is called out rather than inlined.
const (
	ftCfgMode      = 0 + 1
	ftCfgIsStarted = 1 + 1
	ftCfgType      = 2 + 1
	ftCfgID        = 3 + 1
	ftCfgFreq      = 4 + 1
	ftCfgFCoord    = 129 + 1
	ftCfgFAxes     = 130 + 1
	ftCfgFRef      = 136 + 1
	ftCfgKp        = 184 + 1
	ftCfgKi        = 208 + 1
	ftCfgKd        = 232 + 1
	ftCfgXeLimit   = 256 + 1
)

// parseFTSensorConfig decodes the FTSensorGetConfig (0xD4) response.
func parseFTSensorConfig(params []byte) (ftSensorConfig, error) {
	need := 1 + ftSensorConfigLen
	if len(params) < need {
		return ftSensorConfig{}, fmt.Errorf(
			"unexpected F/T config response length, got %d want >= %d", len(params), need)
	}

	readVec := func(at int) forceVec {
		var out forceVec
		for i := range out {
			out[i] = float64(rutils.Float32FromBytesLE(params[at+i*4 : at+i*4+4]))
		}
		return out
	}

	cfg := ftSensorConfig{
		Mode:      params[ftCfgMode],
		IsStarted: params[ftCfgIsStarted] != 0,
		Type:      params[ftCfgType],
		ID:        params[ftCfgID],
		// Integers in this protocol are big-endian; only the floats are LE.
		Freq:    binary.BigEndian.Uint16(params[ftCfgFreq : ftCfgFreq+2]),
		Coord:   params[ftCfgFCoord],
		Ref:     readVec(ftCfgFRef),
		Kp:      readVec(ftCfgKp),
		Ki:      readVec(ftCfgKi),
		Kd:      readVec(ftCfgKd),
		XeLimit: readVec(ftCfgXeLimit),
	}
	for i := range cfg.Axes {
		cfg.Axes[i] = params[ftCfgFAxes+i] != 0
	}
	return cfg, nil
}

// asMap renders the config for a DoCommand response.
func (c ftSensorConfig) asMap() map[string]any {
	vec := func(v forceVec) []any {
		out := make([]any, len(v))
		for i, f := range v {
			out[i] = f
		}
		return out
	}
	axes := make([]any, len(c.Axes))
	for i, on := range c.Axes {
		axes[i] = on
	}
	return map[string]any{
		"mode":       int(c.Mode),
		"mode_name":  ftModeName(c.Mode),
		"is_started": c.IsStarted,
		"type":       int(c.Type),
		"id":         int(c.ID),
		"sample_hz":  int(c.Freq),
		"frame":      frameName(c.Coord),
		"axes":       axes,
		"ref":        vec(c.Ref),
		"kp":         vec(c.Kp),
		"ki":         vec(c.Ki),
		"kd":         vec(c.Kd),
		"xe_limit":   vec(c.XeLimit),
	}
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

func (x *xArm) setFTSensorMode(ctx context.Context, mode byte) error {
	c := x.newCmd(regMap["FTSensorSetMode"])
	c.params = append(c.params, mode)
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) getFTSensorMode(ctx context.Context) (byte, error) {
	c := x.newCmd(regMap["FTSensorGetMode"])
	resp, err := x.send(ctx, c, true)
	if err != nil {
		return 0, err
	}
	if len(resp.params) < 2 {
		return 0, fmt.Errorf("short F/T mode response, got %d bytes", len(resp.params))
	}
	return resp.params[1], nil
}

func (x *xArm) setForceControlConfig(ctx context.Context, coord byte, axes forceAxes, ref forceVec) error {
	c := x.newCmd(regMap["ForceCtrlConfig"])
	c.params = append(c.params, forceCtrlConfigParams(coord, axes, ref)...)
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) setForceControlPID(ctx context.Context, kp, ki, kd, xeLimit forceVec) error {
	c := x.newCmd(regMap["ForceCtrlPID"])
	c.params = append(c.params, forceCtrlPIDParams(kp, ki, kd, xeLimit)...)
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) getFTSensorConfig(ctx context.Context) (ftSensorConfig, error) {
	c := x.newCmd(regMap["FTSensorGetConfig"])
	resp, err := x.send(ctx, c, true)
	if err != nil {
		return ftSensorConfig{}, err
	}
	return parseFTSensorConfig(resp.params)
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// forceControlRequest is one arming of the controller's force mode.
type forceControlRequest struct {
	coord   byte
	axes    forceAxes
	ref     forceVec
	kp      forceVec
	ki      forceVec
	kd      forceVec
	xeLimit forceVec
}

// startForceControl arms hybrid position/force control.
//
// The ordering is the upstream SDK's (example/8003-force_control.cc) and is not
// negotiable: parameters are written first, the sensor stream second, the mode
// third, and the final setMotionState(0) is what actually starts the loop. Skip
// that last re-arm and everything returns success while the arm does nothing.
//
// Two things happen here that are not in the SDK example:
//
// Motion mode is forced to 0. Every UFactory force-control example runs from
// position mode, and servo mode has been reported to ignore compliance
// parameters silently (xArm-Python-SDK issue #146). This driver defaults to mode
// 1, so it has to be moved and then held there -- see moveOptions.
//
// Collision detection is turned off. A contact task trips it on the first touch;
// UFactory's own answer to a sanding case was set_collision_sensitivity(0)
// followed by a state re-arm (xarm_ros issue #248). stopForceControl restores
// the configured value, which is why that restore is mandatory rather than
// best-effort.
//
// Deliberately absent: any call to setFTSensorZero. UFactory disavow it for
// setup -- it compensates from the current readings, so it is invalid as soon as
// the arm's posture changes, and it overwrites an identified payload config.
// Payload identification belongs in UFactory Studio, once.
func (x *xArm) startForceControl(ctx context.Context, req forceControlRequest) error {
	if !req.axes.any() {
		return fmt.Errorf("force control needs at least one compliant axis, got none")
	}

	if err := x.checkReadyState(ctx, true); err != nil {
		return err
	}
	if err := x.start(ctx, true); err != nil { // direct == motion mode 0
		return fmt.Errorf("could not enter position mode for force control: %w", err)
	}
	if err := x.setCollisionDetectionSensitivity(ctx, 0); err != nil {
		return fmt.Errorf("could not disable collision detection: %w", err)
	}
	if err := x.setForceControlPID(ctx, req.kp, req.ki, req.kd, req.xeLimit); err != nil {
		return fmt.Errorf("could not set force control gains: %w", err)
	}
	if err := x.setForceControlConfig(ctx, req.coord, req.axes, req.ref); err != nil {
		return fmt.Errorf("could not set force control target: %w", err)
	}
	if err := x.setFTSensorEnable(ctx, true); err != nil {
		return fmt.Errorf("could not enable the F/T sensor stream: %w", err)
	}
	if err := x.setFTSensorMode(ctx, ftModeForce); err != nil {
		return fmt.Errorf("could not select force control mode: %w", err)
	}
	// This is what starts it.
	if err := x.setMotionState(ctx, 0); err != nil {
		return fmt.Errorf("could not start force control: %w", err)
	}

	x.forceControlActive.Store(true)
	x.logger.Infof("force control armed: frame=%s axes=%v target=%v", frameName(req.coord), req.axes, req.ref)
	return nil
}

// stopForceControl disarms force control and restores collision detection.
//
// Safe to call when force control was never armed, so teardown paths can call it
// unconditionally. Errors are joined rather than returned on the first failure:
// leaving collision detection off is itself a safety regression, so the restore
// is attempted even if clearing the mode failed.
func (x *xArm) stopForceControl(ctx context.Context) error {
	if !x.forceControlActive.Load() {
		return nil
	}
	// Cleared first so that a partial failure below still leaves the driver
	// believing force control is off, which is the safer of the two beliefs:
	// it unpins motion mode and stops blocking manual mode.
	x.forceControlActive.Store(false)

	var errs []error
	if err := x.setFTSensorMode(ctx, ftModeOff); err != nil {
		errs = append(errs, fmt.Errorf("could not clear force control mode: %w", err))
	} else if mode, err := x.getFTSensorMode(ctx); err != nil {
		// Not fatal -- the write above was acknowledged. Worth one round trip to
		// say so, because "we asked it to stop pushing" and "it stopped pushing"
		// are different claims on a path that exists for safety.
		x.logger.Warnf("could not confirm force control was disarmed: %v", err)
	} else if mode != ftModeOff {
		errs = append(errs, fmt.Errorf(
			"force control did not disarm: controller still reports mode %d", mode))
	}
	if err := x.restoreCollisionSensitivity(ctx); err != nil {
		errs = append(errs, err)
	}
	if err := x.setMotionState(ctx, 0); err != nil {
		errs = append(errs, fmt.Errorf("could not re-arm motion state: %w", err))
	}

	if len(errs) > 0 {
		return errors.Join(errs...)
	}
	x.logger.Info("force control disarmed")
	return nil
}

// restoreCollisionSensitivity puts collision detection back to the configured
// value after force control turned it off.
func (x *xArm) restoreCollisionSensitivity(ctx context.Context) error {
	if x.conf == nil || x.conf.Sensitivity == nil {
		// Nothing configured means the controller's own default was in force
		// before we zeroed it. The module's default is 3; use it rather than
		// leaving detection disabled.
		if err := x.setCollisionDetectionSensitivity(ctx, defaultCollisionSensitivity); err != nil {
			return fmt.Errorf("could not restore default collision detection: %w", err)
		}
		return nil
	}
	if err := x.setCollisionDetectionSensitivity(ctx, *x.conf.Sensitivity); err != nil {
		return fmt.Errorf("could not restore collision detection: %w", err)
	}
	return nil
}

func frameName(coord byte) string {
	if coord == forceFrameTool {
		return forceFrameToolName
	}
	return forceFrameBaseName
}

// ftModeName renders the controller's compliance mode. A caller diagnosing
// "why is the arm not holding force" needs to distinguish "off" from
// "admittance" -- both leave the target force unheld, for different reasons.
func ftModeName(mode byte) string {
	switch mode {
	case ftModeOff:
		return "off"
	case ftModeAdmittance:
		return "admittance"
	case ftModeForce:
		return "force"
	default:
		return fmt.Sprintf("unknown(%d)", mode)
	}
}

// ---------------------------------------------------------------------------
// DoCommand parsing
// ---------------------------------------------------------------------------

// axisNames maps the shorthand accepted by the `axes` field to its index.
var axisNames = map[string]int{"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}

// parseForceControlRequest builds a request from the `set_force_control` value.
//
// Accepted shapes, all going over the wire as JSON so numbers arrive as float64:
//
//	{"axes": "z", "forces": -5}                       // shorthand: one axis, one force
//	{"axes": ["z"], "forces": {"z": -5}}              // named
//	{"axes": [false,false,true,false,false,false],
//	 "forces": [0,0,-5,0,0,0]}                        // explicit 6-vectors
//
// plus optional "frame" ("base" or "tool", default base) and the gains
// "kp"/"ki"/"kd"/"max_velocity", each a single number applied to all six axes.
func parseForceControlRequest(val any) (forceControlRequest, error) {
	req := forceControlRequest{
		coord:   forceFrameBase,
		kp:      uniformVec(defaultForceKp),
		ki:      uniformVec(defaultForceKi),
		kd:      uniformVec(defaultForceKd),
		xeLimit: uniformVec(defaultForceXeLimit),
	}

	m, ok := val.(map[string]any)
	if !ok {
		return req, fmt.Errorf("%s wants a map, got %T", setForceControlKey, val)
	}

	axes, err := parseForceAxes(m["axes"])
	if err != nil {
		return req, err
	}
	req.axes = axes

	ref, err := parseForceRef(m["forces"], axes)
	if err != nil {
		return req, err
	}
	req.ref = ref

	if f, present := m["frame"]; present {
		name, isStr := f.(string)
		if !isStr {
			return req, fmt.Errorf("frame wants a string, got %T", f)
		}
		switch name {
		case forceFrameBaseName:
			req.coord = forceFrameBase
		case forceFrameToolName:
			req.coord = forceFrameTool
		default:
			return req, fmt.Errorf("frame must be %q or %q, got %q",
				forceFrameBaseName, forceFrameToolName, name)
		}
	}

	for _, g := range []struct {
		key string
		dst *forceVec
	}{{"kp", &req.kp}, {"ki", &req.ki}, {"kd", &req.kd}, {"max_velocity", &req.xeLimit}} {
		if v, present := m[g.key]; present {
			f, isNum := toFloat(v)
			if !isNum {
				return req, fmt.Errorf("%s wants a number, got %T", g.key, v)
			}
			*g.dst = uniformVec(f)
		}
	}

	return req, nil
}

// parseForceAxes accepts "z", ["z","rx"], or an explicit 6-element bool list.
func parseForceAxes(v any) (forceAxes, error) {
	var axes forceAxes
	switch t := v.(type) {
	case nil:
		return axes, fmt.Errorf("axes is required: name the compliant axes, e.g. \"z\"")

	case string:
		i, known := axisNames[t]
		if !known {
			return axes, fmt.Errorf("unknown axis %q, want one of x y z rx ry rz", t)
		}
		axes[i] = true
		return axes, nil

	case []any:
		if len(t) == 6 && isAllBool(t) {
			for i, e := range t {
				axes[i], _ = e.(bool)
			}
			return axes, nil
		}
		for _, e := range t {
			name, isStr := e.(string)
			if !isStr {
				return axes, fmt.Errorf(
					"axes list wants axis names or exactly 6 booleans, got %T in the list", e)
			}
			i, known := axisNames[name]
			if !known {
				return axes, fmt.Errorf("unknown axis %q, want one of x y z rx ry rz", name)
			}
			axes[i] = true
		}
		return axes, nil

	default:
		return axes, fmt.Errorf("axes wants a name, a list of names, or 6 booleans, got %T", v)
	}
}

// parseForceRef accepts a bare number (applied to the single compliant axis), a
// map keyed by axis name, or an explicit 6-element list.
func parseForceRef(v any, axes forceAxes) (forceVec, error) {
	var ref forceVec

	switch t := v.(type) {
	case nil:
		return ref, fmt.Errorf("forces is required: give the target force per compliant axis")

	case []any:
		if len(t) != 6 {
			return ref, fmt.Errorf("forces as a list wants exactly 6 entries, got %d", len(t))
		}
		for i, e := range t {
			f, isNum := toFloat(e)
			if !isNum {
				return ref, fmt.Errorf("forces[%d] wants a number, got %T", i, e)
			}
			ref[i] = f
		}

	case map[string]any:
		for name, e := range t {
			i, known := axisNames[name]
			if !known {
				return ref, fmt.Errorf("unknown axis %q in forces, want one of x y z rx ry rz", name)
			}
			f, isNum := toFloat(e)
			if !isNum {
				return ref, fmt.Errorf("forces[%q] wants a number, got %T", name, e)
			}
			ref[i] = f
		}

	default:
		f, isNum := toFloat(v)
		if !isNum {
			return ref, fmt.Errorf("forces wants a number, a map or a 6-element list, got %T", v)
		}
		// A bare number is only unambiguous with exactly one compliant axis.
		// Collected rather than counted with a sentinel index: the length check
		// below then proves the subscript is in range, to a reader and to gosec
		// alike.
		var compliant []int
		for i, on := range axes {
			if on {
				compliant = append(compliant, i)
			}
		}
		if len(compliant) != 1 {
			return ref, fmt.Errorf(
				"forces as a single number needs exactly one compliant axis, but %d are set; "+
					"give a map or a 6-element list instead", len(compliant))
		}
		ref[compliant[0]] = f
	}

	// A compliant axis with no target would be driven to zero force, which is
	// almost never what a caller means and looks like the arm ignoring them.
	for i, on := range axes {
		if on && ref[i] == 0 {
			return ref, fmt.Errorf(
				"axis %s is compliant but its target force is 0; "+
					"give it a non-zero force, or drop it from axes", axisName(i))
		}
	}
	return ref, nil
}

func axisName(i int) string {
	for name, idx := range axisNames {
		if idx == i {
			return name
		}
	}
	return fmt.Sprintf("axis%d", i)
}

func isAllBool(vals []any) bool {
	for _, v := range vals {
		if _, ok := v.(bool); !ok {
			return false
		}
	}
	return true
}

// toFloat accepts the numeric types that survive a JSON/protobuf round trip.
func toFloat(v any) (float64, bool) {
	switch t := v.(type) {
	case float64:
		return t, true
	case float32:
		return float64(t), true
	case int:
		return float64(t), true
	case int64:
		return float64(t), true
	default:
		return 0, false
	}
}
