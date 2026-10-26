package arm

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
)

// tcpPayload is the load on the arm's flange (gripper, tooling and any held
// part), which the controller uses for gravity compensation in manual mode and
// for collision detection.
type tcpPayload struct {
	MassKg float64
	// CenterOfGravityMM is in mm, relative to the flange in the tool frame.
	CenterOfGravityMM r3.Vector
}

// presetToleranceKg is how far the controller's payload mass may be from the
// mounted gripper's preset and still count as that preset. Studio presets,
// spec-sheet weights and the report's float32 values differ by tens of grams.
const presetToleranceKg = 0.1

// gripperPreset is the payload a UFACTORY gripper puts on the flange. It is only
// applied when the controller reports no payload at all.
type gripperPreset struct {
	label   string
	payload tcpPayload
}

// The xArm Gripper G1 and Vacuum Gripper values are UFACTORY Studio's built-in
// presets (Studio user manual V2.6.0). Studio publishes no G2 preset; until one
// is read off a G2 it uses the G1 values, which match its 0.8 kg spec weight.
// UFACTORY publishes no preset for the Lite 6 grippers, so theirs come from the
// inertial blocks of the bundled URDFs.
var (
	presetGripperG1         = gripperPreset{"xArm Gripper G1", tcpPayload{MassKg: 0.82, CenterOfGravityMM: r3.Vector{Z: 48}}}
	presetGripperG2         = gripperPreset{"xArm Gripper G2", tcpPayload{MassKg: 0.82, CenterOfGravityMM: r3.Vector{Z: 48}}}
	presetVacuumGripper     = gripperPreset{"xArm Vacuum Gripper", tcpPayload{MassKg: 0.61, CenterOfGravityMM: r3.Vector{Z: 53}}}
	presetGripperLite       = gripperPreset{"Lite 6 Gripper", tcpPayload{MassKg: 0.25, CenterOfGravityMM: r3.Vector{Z: 30}}}
	presetVacuumGripperLite = gripperPreset{"Lite 6 Vacuum Gripper", tcpPayload{MassKg: 0.14, CenterOfGravityMM: r3.Vector{Z: 30}}}
)

// presetFor returns the preset for a gripper model, using the detected
// submodel to tell the G1 and G2 apart.
func presetFor(gripperModel string, detected detectedGripper) (gripperPreset, bool) {
	switch gripperModel {
	case ModelNameGripper:
		if detected.submodel == submodelG2 {
			return presetGripperG2, true
		}
		return presetGripperG1, true
	case ModelNameVacuumGripper:
		return presetVacuumGripper, true
	case ModelNameGripperLite:
		return presetGripperLite, true
	case ModelNameVacuumGripperLite:
		return presetVacuumGripperLite, true
	default:
		return gripperPreset{}, false
	}
}

// registerGripperPreset tells the arm which UFACTORY gripper is mounted, so the
// arm's next payload check can fall back to its preset. Nothing is sent here,
// so a gripper never switches controller modes underneath a running motion.
// The check runs on the next manual-mode entry, or on the next start() that
// re-asserts the motion mode (after Stop, a cleared error, leaving manual mode,
// or a reconnect); start() skips it while the arm is already in that mode. It
// returns a release func for the gripper's Close.
func registerGripperPreset(a arm.Arm, gripperModel string, detected detectedGripper, logger logging.Logger) func() {
	noop := func() {}
	p, ok := presetFor(gripperModel, detected)
	if !ok {
		return noop
	}
	x, ok := a.(*xArm)
	if !ok {
		logger.Debugf("arm is not a local xArm; not registering the %s payload preset", p.label)
		return noop
	}

	x.gripperPreset.Store(&p)
	x.payloadChecked.Store(false)
	logger.Debugf("registered the %s payload preset on arm %s", p.label, x.name.Name)

	// Compare-and-swap so closing a gripper never clears a preset that a newer
	// gripper on the same arm has since registered.
	return func() { x.gripperPreset.CompareAndSwap(&p, nil) }
}

// decideTCPPayload returns the gripper preset to send, or nil to leave the
// controller's payload alone. UFACTORY Studio owns the payload: a non-zero
// value on the controller (a Studio preset, payload identification, or the
// saved config) is never overwritten. A zero mass counts as unset, and with a
// UFACTORY gripper mounted it is never correct, so the gripper's preset fills
// in.
//
// Every call logs the payload the arm will assume, and warns when that is not
// the mounted gripper's preset, since a wrong payload makes the arm drift or
// resist in manual mode. reported is nil when the controller's payload could
// not be read.
func decideTCPPayload(reported *tcpPayload, preset *gripperPreset, logger logging.Logger) *tcpPayload {
	switch {
	case reported == nil:
		logger.Warn("controller's TCP payload is unknown; leaving it unchanged")
		return nil
	case reported.MassKg <= 0 && preset != nil:
		logger.Infof("controller has no TCP payload set; applying the %s preset %s", preset.label, formatPayload(&preset.payload))
		return &preset.payload
	case reported.MassKg <= 0:
		logger.Info("controller has no TCP payload set and no UFACTORY gripper is registered with this arm; " +
			"the arm assumes nothing is mounted on the end effector")
		return nil
	case preset != nil && math.Abs(reported.MassKg-preset.payload.MassKg) <= presetToleranceKg:
		logger.Infof("the arm assumes TCP payload %s, which matches the %s preset", formatPayload(reported), preset.label)
		return nil
	default:
		expected := "no UFACTORY gripper is registered with this arm"
		if preset != nil {
			expected = fmt.Sprintf("the %s preset is %.3fkg", preset.label, preset.payload.MassKg)
		}
		logger.Warnf("the arm assumes TCP payload %s is mounted on the end effector, but %s. "+
			"If that is not what is mounted, set the payload in UFACTORY Studio; "+
			"a wrong payload makes the arm drift or resist in manual mode.",
			formatPayload(reported), expected)
		return nil
	}
}

func formatPayload(p *tcpPayload) string {
	if p == nil {
		return "unknown"
	}
	return fmt.Sprintf("mass=%.3fkg center_of_gravity=(%.1f, %.1f, %.1f)mm",
		p.MassKg, p.CenterOfGravityMM.X, p.CenterOfGravityMM.Y, p.CenterOfGravityMM.Z)
}

// reportedTCPLoad returns the controller's current payload from the report
// stream. Right after startup the first packet may still be in flight, so it
// waits briefly for it; a stream that has gone stale fails at once.
func (x *xArm) reportedTCPLoad(ctx context.Context) (reportedTCPLoad, error) {
	load, err := x.report.tcpLoad()
	if !errors.Is(err, errNoReport) || x.report == nil {
		return load, err
	}
	if err := x.report.waitForFirstReport(ctx, reportFirstTimeout); err != nil {
		return reportedTCPLoad{}, err
	}
	return x.report.tcpLoad()
}

// ensureTCPPayload reads the controller's payload and sends the gripper
// preset if decideTCPPayload picks it. Sending requires mode 0 / state 0 (see
// setTCPPayload); pass inPositionMode when the caller has already put the arm
// there, otherwise the mode is switched only if something is actually sent.
func (x *xArm) ensureTCPPayload(ctx context.Context, inPositionMode bool) error {
	var reported *tcpPayload
	if load, err := x.reportedTCPLoad(ctx); err != nil {
		x.logger.Warnf("could not read the controller's TCP payload: %v", err)
	} else {
		reported = &tcpPayload{MassKg: load.MassKg, CenterOfGravityMM: load.CenterOfGravityMM}
	}

	if p := decideTCPPayload(reported, x.gripperPreset.Load(), x.logger); p != nil {
		if !inPositionMode {
			if err := x.setMotionMode(ctx, 0); err != nil {
				return err
			}
			if err := x.setMotionState(ctx, 0); err != nil {
				return err
			}
		}
		if err := x.setTCPPayload(ctx, *p); err != nil {
			return err
		}
	}
	x.payloadChecked.Store(true)
	return nil
}
