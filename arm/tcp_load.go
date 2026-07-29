package arm

// This file supports the xArm controller's TCP payload (load) setting.
//
// The controller uses payload mass and center of gravity to compute its dynamic
// model, which drives gravity compensation, collision detection, and manual
// (drag-teach) mode. A payload that does not match reality produces false
// collision trips and drift in manual mode.

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"strconv"
	"strings"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils"
)

// tcpLoad is a payload description: mass at a center of gravity expressed in
// the controller's default TCP frame, which sits at the flange center. Because
// that frame is fixed at the flange, cogMM is independent of any TCP offset and
// of whatever tool frame Viam has configured — no transform is ever needed.
type tcpLoad struct {
	massKg float64
	cogMM  r3.Vector
}

// validate rejects values that are malformed rather than merely unusual. Mass
// above the arm's rating is a separate, softer check — see ratedPayloadKg.
func (l tcpLoad) validate() error {
	if math.IsNaN(l.massKg) || math.IsInf(l.massKg, 0) {
		return fmt.Errorf("tcp load mass must be a finite number, got %v", l.massKg)
	}
	if l.massKg < 0 {
		return fmt.Errorf("tcp load mass cannot be negative, got %v", l.massKg)
	}
	if math.IsInf(float64(float32(l.massKg)), 0) {
		return fmt.Errorf("tcp load mass is not representable as float32, got %v", l.massKg)
	}
	for _, c := range []struct {
		name string
		v    float64
	}{{"x", l.cogMM.X}, {"y", l.cogMM.Y}, {"z", l.cogMM.Z}} {
		if math.IsNaN(c.v) || math.IsInf(c.v, 0) {
			return fmt.Errorf("tcp load center of gravity %s must be a finite number, got %v", c.name, c.v)
		}
		if math.IsInf(float64(float32(c.v)), 0) {
			return fmt.Errorf("tcp load center of gravity %s is not representable as float32, got %v", c.name, c.v)
		}
	}
	return nil
}

// minMillimeterFirmware is the oldest firmware that expects CoG in mm.
var minMillimeterFirmware = [3]int{0, 2, 1}

// firmwareUsesMillimeters reports whether the controller expects the center of
// gravity in mm. Firmware >= minMillimeterFirmware uses mm; older firmware
// uses meters (the xArm SDK divides by 1000 below that version).
//
// Unknown or unparseable versions default to mm. Every deployed controller is
// far above minMillimeterFirmware, so mm is the safe guess; assuming meters
// would under-report the center of gravity by 1000x on the overwhelmingly
// likely path.
func firmwareUsesMillimeters(version string) bool {
	parts := strings.Split(version, ".")
	if len(parts) != 3 {
		return true
	}
	nums := make([]int, 3)
	for i, p := range parts {
		n, err := strconv.Atoi(strings.TrimSpace(p))
		if err != nil || n < 0 {
			return true
		}
		nums[i] = n
	}
	for i, want := range minMillimeterFirmware {
		if nums[i] != want {
			return nums[i] > want
		}
	}
	return true // exactly minMillimeterFirmware
}

// encodeTCPLoad packs the four little-endian float32 values the controller
// expects for SET_LOAD_PARAM: [mass, cx, cy, cz].
//
// When useMM is false the center of gravity is converted to meters; mass is
// always kilograms regardless of firmware.
func encodeTCPLoad(l tcpLoad, useMM bool) []byte {
	scale := 1.0
	if !useMM {
		scale = 0.001
	}
	out := make([]byte, 16)
	for i, v := range [...]float64{l.massKg, l.cogMM.X * scale, l.cogMM.Y * scale, l.cogMM.Z * scale} {
		binary.LittleEndian.PutUint32(out[i*4:], math.Float32bits(float32(v)))
	}
	return out
}

// ratedPayloadKg returns the manufacturer's rated payload for a model. The
// second return is false when the model is unknown, in which case no rating
// check can be made.
func ratedPayloadKg(m hardwareModel) (float64, bool) {
	switch m {
	case hardwareModelLite6:
		return 0.5, true
	case hardwareModelXArm5:
		return 3.0, true
	case hardwareModelXArm7, hardwareModelXArm7T:
		return 3.5, true
	case hardwareModelXArm6, hardwareModelXArm850:
		return 5.0, true
	case hardwareModelUnknown:
		return 0, false
	default:
		return 0, false
	}
}

// gripperDefaultTCPLoad returns the payload preset for a gripper's registered
// resource model, and whether one exists.
//
// Keyed on the registered model rather than the Go type or the probe result,
// deliberately:
//
//   - newVacuumGripper is the constructor for BOTH VacuumGripperModel and
//     VacuumGripperModelLite, so the Go type cannot tell them apart.
//   - probeGripper returns submodel "" on any probe error, and
//     vacuumGripperSubmodel falls back to v1 when arm detection failed, so the
//     detected submodel fails toward "not lite" — which would push 0.61 kg onto
//     a Lite6 rated for 0.5 kg total.
//
// config.Model is config-derived and cannot fail, so it is the only signal here.
//
// The Lite variants have no published UFactory preset and intentionally return
// false. The xArm BIO Gripper preset (0.72 kg) is also absent: no component in
// this module is specific to it — myGripperLite is the sole consumer of
// detectBioGripper. Add it if a distinct bio-gripper component ever exists.
func gripperDefaultTCPLoad(model resource.Model) (tcpLoad, bool) {
	switch model {
	case GripperModel:
		return tcpLoad{massKg: 0.82, cogMM: r3.Vector{Z: 48}}, true
	case VacuumGripperModel:
		return tcpLoad{massKg: 0.61, cogMM: r3.Vector{Z: 53}}, true
	default:
		return tcpLoad{}, false
	}
}

// TCPLoadConfig is the JSON shape of the `tcp_load` config attribute.
type TCPLoadConfig struct {
	// MassKg is a pointer so an explicit `"mass_kg": 0` (a genuine zero payload)
	// can be told apart from an omitted field (a mistake: the user described a
	// center of gravity but forgot the mass). Both would otherwise decode to the
	// same zero value and silently write a 0 kg payload.
	MassKg *float64 `json:"mass_kg"`
	// CenterOfGravityMM is [x, y, z] relative to the flange center. Optional;
	// omitting it means the origin.
	CenterOfGravityMM []float64 `json:"center_of_gravity_mm,omitempty"`
}

func (c *TCPLoadConfig) toTCPLoad() (tcpLoad, error) {
	if c.MassKg == nil {
		return tcpLoad{}, fmt.Errorf("tcp_load.mass_kg is required")
	}
	l := tcpLoad{massKg: *c.MassKg}
	switch len(c.CenterOfGravityMM) {
	case 0:
		// omitted; cogMM stays at the origin
	case 3:
		l.cogMM = r3.Vector{X: c.CenterOfGravityMM[0], Y: c.CenterOfGravityMM[1], Z: c.CenterOfGravityMM[2]}
	default:
		return tcpLoad{}, fmt.Errorf(
			"tcp_load.center_of_gravity_mm must have exactly 3 elements [x, y, z], got %d",
			len(c.CenterOfGravityMM),
		)
	}
	if err := l.validate(); err != nil {
		return tcpLoad{}, fmt.Errorf("tcp_load: %w", err)
	}
	return l, nil
}

// applyConfigTCPLoad applies a config-sourced payload. apply is injected so the
// source/requester wiring is testable without a controller.
func applyConfigTCPLoad(ctx context.Context, c *TCPLoadConfig,
	apply func(context.Context, tcpLoad, tcpLoadSource, string) error,
) error {
	if c == nil {
		return nil
	}
	l, err := c.toTCPLoad()
	if err != nil {
		return err
	}
	return apply(ctx, l, tcpLoadSourceConfig, "config")
}

// tcpLoadSource records where the currently-cached payload came from. It is
// what enforces precedence between config, runtime writes, and gripper defaults.
type tcpLoadSource int

const (
	tcpLoadSourceUnset          tcpLoadSource = iota // this module has written nothing
	tcpLoadSourceConfig                              // the tcp_load config attribute
	tcpLoadSourceDoCommand                           // a runtime set_tcp_load
	tcpLoadSourceGripperDefault                      // pushed by a gripper component
)

func (s tcpLoadSource) String() string {
	switch s {
	case tcpLoadSourceConfig:
		return "config"
	case tcpLoadSourceDoCommand:
		return "do_command"
	case tcpLoadSourceGripperDefault:
		return "gripper_default"
	case tcpLoadSourceUnset:
		return "unset"
	default:
		return fmt.Sprintf("unknown(%d)", int(s))
	}
}

// shouldApplyTCPLoad decides whether a write from `incoming` may overwrite a
// payload currently sourced from `current`.
//
// Explicit writes (config, set_tcp_load) always apply. A gripper default
// applies only when this module has written nothing at all.
//
// That asymmetry is the point. Gripper components are AlwaysRebuild and are
// reconstructed whenever their OWN config changes (gripper_speed,
// vacuum_length_mm, connection_type) with no arm change involved. Without this
// rule, editing vacuum_length_mm while the arm holds a 1.2 kg workpiece would
// push the 0.61 kg gripper default and leave the controller compensating for
// the wrong mass under load.
//
// Only a new arm instance re-arms defaults: the arm is AlwaysRebuild, so an arm
// config change resets the cache to unset and the next gripper push applies.
func shouldApplyTCPLoad(current, incoming tcpLoadSource) bool {
	if incoming == tcpLoadSourceGripperDefault {
		return current == tcpLoadSourceUnset
	}
	return true
}

// setTCPLoad writes the payload to the controller. It does not validate, check
// the arm's rating, or update the cache — callers own that policy.
func (x *xArm) setTCPLoad(ctx context.Context, l tcpLoad) error {
	c := x.newCmd(regMap["SetTCPLoad"])
	c.params = append(c.params, encodeTCPLoad(l, firmwareUsesMillimeters(x.detectedArm.firmwareVersion))...)
	_, err := x.send(ctx, c, true)
	return err
}

// exceedsRating reports whether a payload is above the model's rated capacity,
// and what that rating is. Always false for unknown models.
func exceedsRating(l tcpLoad, m hardwareModel) (bool, float64) {
	rated, ok := ratedPayloadKg(m)
	if !ok {
		return false, 0
	}
	return l.massKg > rated, rated
}

// tcpLoadAction is what decideTCPLoad concluded should happen to a write.
type tcpLoadAction int

const (
	tcpLoadActionApply    tcpLoadAction = iota // write it
	tcpLoadActionSuppress                      // precedence: something already set the payload
	tcpLoadActionRefuse                        // a pushed default above the arm's rating
)

// tcpLoadDecision is the policy outcome for one write, computed without I/O so
// it can be tested exhaustively.
type tcpLoadDecision struct {
	action         tcpLoadAction
	warnOverRating bool // apply, but the mass exceeds the rating
	ratedKg        float64
}

// decideTCPLoad applies precedence first, then the rating rule.
//
// Over-rating is a warning for explicit writes (config, set_tcp_load) — the
// user typed that number and may know something we don't — but a hard refusal
// for a pushed default, which nobody typed. That refusal is the independent
// second guard against a mis-keyed default reaching a Lite6.
func decideTCPLoad(l tcpLoad, incoming, current tcpLoadSource, model hardwareModel) tcpLoadDecision {
	if !shouldApplyTCPLoad(current, incoming) {
		return tcpLoadDecision{action: tcpLoadActionSuppress}
	}
	over, rated := exceedsRating(l, model)
	if !over {
		return tcpLoadDecision{action: tcpLoadActionApply}
	}
	if incoming == tcpLoadSourceGripperDefault {
		return tcpLoadDecision{action: tcpLoadActionRefuse, ratedKg: rated}
	}
	return tcpLoadDecision{action: tcpLoadActionApply, warnOverRating: true, ratedKg: rated}
}

// applyTCPLoad validates, checks precedence and rating, writes to the
// controller, and updates the cache. It is the single entry point for every
// payload write; nothing else should call setTCPLoad directly.
//
// See applyTCPLoadWith for the locking contract.
func (x *xArm) applyTCPLoad(ctx context.Context, l tcpLoad, src tcpLoadSource, requester string) error {
	return x.applyTCPLoadWith(ctx, l, src, requester, x.setTCPLoad)
}

// applyTCPLoadWith is applyTCPLoad with the controller write injected, so the
// policy and caching behavior can be tested without a socket.
//
// Locking contract: caller must NOT hold confLock. This method takes
// tcpLoadApplyLock for its entire body, including across the controller
// write, and releases it on return. That makes the decide→write→cache
// sequence atomic against a second, concurrent applier: without it, two
// callers can both read the same stale `current`, both decide Apply, and
// interleave their writes so the cache ends up disagreeing with whichever
// write actually reached the controller last. There is no read-back
// register, so that disagreement would be silent and permanent — the exact
// mid-grasp corruption the precedence rule in shouldApplyTCPLoad exists to
// prevent. Only payload writes ever contend on this lock, and they are rare,
// so holding it across the (Modbus) write is an acceptable cost.
func (x *xArm) applyTCPLoadWith(
	ctx context.Context,
	l tcpLoad,
	src tcpLoadSource,
	requester string,
	write func(context.Context, tcpLoad) error,
) error {
	if err := l.validate(); err != nil {
		return err
	}

	x.tcpLoadApplyLock.Lock()
	defer x.tcpLoadApplyLock.Unlock()

	x.confLock.Lock()
	current, currentRequester, currentMassKg := x.tcpLoadSource, x.tcpLoadRequester, x.tcpLoad.massKg
	x.confLock.Unlock()

	// detectedArm is written exactly once, during construction, before the arm
	// is published (see the field comment in xarm.go), so reading it here
	// without confLock is safe. Captured once since the value is used below in
	// more than one place.
	model := x.detectedArm.model

	switch d := decideTCPLoad(l, src, current, model); d.action {
	case tcpLoadActionSuppress:
		// Two gripper components on one arm is a config the user should fix, and
		// which one wins depends on non-deterministic construction order — so warn,
		// name both, and report the mass currently in effect. Any other suppression
		// is the rule working as intended.
		if current == tcpLoadSourceGripperDefault {
			x.logger.Warnf(
				"tcp load default from %q (%.3f kg) ignored: %q already set a gripper default of %.3f kg; "+
					"two grippers on one arm is ambiguous, set tcp_load on the arm to be explicit",
				requester, l.massKg, currentRequester, currentMassKg,
			)
		} else {
			x.logger.Infof(
				"tcp load default from %q (%.3f kg, cog [%.2f %.2f %.2f] mm) not applied: "+
					"payload already set by %q (source: %s)",
				requester, l.massKg, l.cogMM.X, l.cogMM.Y, l.cogMM.Z, currentRequester, current,
			)
		}
		return nil
	case tcpLoadActionRefuse:
		x.logger.Warnf(
			"tcp load default from %q (%.3f kg) exceeds the %s rated payload of %.3f kg; not applying",
			requester, l.massKg, model, d.ratedKg,
		)
		return nil
	case tcpLoadActionApply:
		switch {
		case d.warnOverRating:
			x.logger.Warnf(
				"tcp load %.3f kg from %q exceeds the %s rated payload of %.3f kg; applying anyway",
				l.massKg, requester, model, d.ratedKg,
			)
		default:
			if _, ok := ratedPayloadKg(model); !ok {
				x.logger.Debugf(
					"tcp load %.3f kg from %q applied without a rating check: %s rated payload is unknown",
					l.massKg, requester, model,
				)
			}
		}
	default:
		// decideTCPLoad only ever returns the three actions above; this exists so
		// a future action added there fails loudly here instead of silently
		// falling out of the switch and reaching the controller unchecked.
		return fmt.Errorf("unhandled tcp load decision %d for %q", d.action, requester)
	}

	if err := write(ctx, l); err != nil {
		return err
	}

	// Cache only after the controller accepts, so get_tcp_load never reports a
	// value the write refused.
	x.confLock.Lock()
	x.tcpLoad, x.tcpLoadSource, x.tcpLoadRequester = l, src, requester
	x.confLock.Unlock()

	x.logger.Infof(
		"tcp load set to %.3f kg, cog [%.2f %.2f %.2f] mm (source: %s, requester: %q)",
		l.massKg, l.cogMM.X, l.cogMM.Y, l.cogMM.Z, src, requester,
	)
	return nil
}

// parseTCPLoadRequest reads the nested-map payload shared by set_tcp_load and
// set_default_tcp_load. JSON numbers arrive as float64 over the wire.
func parseTCPLoadRequest(params map[string]any) (tcpLoad, error) {
	raw, ok := params["mass_kg"]
	if !ok {
		return tcpLoad{}, errors.New("tcp load request requires mass_kg")
	}
	mass, err := utils.AssertType[float64](raw)
	if err != nil {
		return tcpLoad{}, fmt.Errorf("tcp load mass_kg: %w", err)
	}
	l := tcpLoad{massKg: mass}

	if rawCog, ok := params["center_of_gravity_mm"]; ok {
		items, err := utils.AssertType[[]any](rawCog)
		if err != nil {
			return tcpLoad{}, fmt.Errorf("tcp load center_of_gravity_mm: %w", err)
		}
		if len(items) != 3 {
			return tcpLoad{}, fmt.Errorf(
				"tcp load center_of_gravity_mm must have exactly 3 elements [x, y, z], got %d", len(items))
		}
		vals := make([]float64, 3)
		for i, item := range items {
			v, err := utils.AssertType[float64](item)
			if err != nil {
				return tcpLoad{}, fmt.Errorf("tcp load center_of_gravity_mm[%d]: %w", i, err)
			}
			vals[i] = v
		}
		l.cogMM = r3.Vector{X: vals[0], Y: vals[1], Z: vals[2]}
	}

	if err := l.validate(); err != nil {
		return tcpLoad{}, err
	}
	return l, nil
}

// tcpLoadResponse renders the cached payload for get_tcp_load. When nothing has
// been written the numeric fields are omitted entirely — reporting 0 kg would
// be indistinguishable from a real zero payload.
func (x *xArm) tcpLoadResponse() map[string]any {
	x.confLock.Lock()
	l, src := x.tcpLoad, x.tcpLoadSource
	x.confLock.Unlock()

	resp := map[string]any{"source": src.String()}
	if src == tcpLoadSourceUnset {
		return resp
	}
	resp["mass_kg"] = l.massKg
	resp["center_of_gravity_mm"] = []float64{l.cogMM.X, l.cogMM.Y, l.cogMM.Z}
	return resp
}
