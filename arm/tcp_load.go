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
	"fmt"
	"math"
	"strconv"
	"strings"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/resource"
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
	for _, c := range []struct {
		name string
		v    float64
	}{{"x", l.cogMM.X}, {"y", l.cogMM.Y}, {"z", l.cogMM.Z}} {
		if math.IsNaN(c.v) || math.IsInf(c.v, 0) {
			return fmt.Errorf("tcp load center of gravity %s must be a finite number, got %v", c.name, c.v)
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
	out := make([]byte, 0, 16)
	for _, v := range []float64{l.massKg, l.cogMM.X * scale, l.cogMM.Y * scale, l.cogMM.Z * scale} {
		b := make([]byte, 4)
		binary.LittleEndian.PutUint32(b, math.Float32bits(float32(v)))
		out = append(out, b...)
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

// setTCPLoad writes the payload to the controller. It does not validate, check
// the arm's rating, or update the cache — callers own that policy.
func (x *xArm) setTCPLoad(ctx context.Context, l tcpLoad) error {
	c := x.newCmd(regMap["SetTCPLoad"])
	c.params = append(c.params, encodeTCPLoad(l, firmwareUsesMillimeters(x.detectedArm.firmwareVersion))...)
	_, err := x.send(ctx, c, true)
	return err
}
