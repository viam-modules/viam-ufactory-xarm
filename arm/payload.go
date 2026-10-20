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

// gripperPreset is the payload a UFACTORY gripper puts on the flange. It is only
// applied when the controller reports no payload at all.
type gripperPreset struct {
	label   string
	payload TCPPayloadConfig
}

// The xArm Gripper G1 and Vacuum Gripper values are UFACTORY Studio's built-in
// presets (Studio user manual V2.6.0). Studio publishes no G2 preset; until one
// is read off a G2 it uses the G1 values, which match its 0.8 kg spec weight.
// UFACTORY publishes no preset for the Lite 6 grippers, so theirs come from the
// inertial blocks of the bundled URDFs.
var (
	presetGripperG1         = gripperPreset{"xArm Gripper G1", TCPPayloadConfig{MassKg: 0.82, CenterOfGravityMM: r3.Vector{Z: 48}}}
	presetGripperG2         = gripperPreset{"xArm Gripper G2", TCPPayloadConfig{MassKg: 0.82, CenterOfGravityMM: r3.Vector{Z: 48}}}
	presetVacuumGripper     = gripperPreset{"xArm Vacuum Gripper", TCPPayloadConfig{MassKg: 0.61, CenterOfGravityMM: r3.Vector{Z: 53}}}
	presetGripperLite       = gripperPreset{"Lite 6 Gripper", TCPPayloadConfig{MassKg: 0.25, CenterOfGravityMM: r3.Vector{Z: 30}}}
	presetVacuumGripperLite = gripperPreset{"Lite 6 Vacuum Gripper", TCPPayloadConfig{MassKg: 0.14, CenterOfGravityMM: r3.Vector{Z: 30}}}
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
// arm's next payload check can fall back to its preset. Nothing is sent here:
// the check runs on the arm's next start() or manual-mode entry, so a gripper
// never switches controller modes underneath a running motion. It returns a
// release func for the gripper's Close.
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

// decideTCPPayload picks the payload to send, or nil to leave the controller's
// alone. In order:
//  1. An explicit tcp_payload in the arm config always wins.
//  2. Otherwise a non-zero payload on the controller was set by the user (a
//     UFACTORY Studio preset, payload identification, or the saved config) and
//     is never overwritten.
//  3. Otherwise the mounted gripper's preset is applied. A zero mass counts as
//     unset: with a UFACTORY gripper mounted it is never correct.
//
// reported is nil when the controller's payload could not be read.
func decideTCPPayload(configured, reported *TCPPayloadConfig, preset *gripperPreset, logger logging.Logger) *TCPPayloadConfig {
	switch {
	case configured != nil:
		logger.Infof("using tcp_payload from the arm config %s; controller has %s", formatPayload(configured), formatPayload(reported))
		warnIfLighterThanPreset("configured tcp_payload", configured, preset, logger)
		return configured
	case reported == nil:
		logger.Warn("controller's TCP payload is unknown; leaving it unchanged")
		return nil
	case reported.MassKg > 0:
		logger.Infof("controller already has TCP payload %s; not overriding it", formatPayload(reported))
		warnIfLighterThanPreset("controller TCP payload", reported, preset, logger)
		return nil
	case preset != nil:
		logger.Infof("controller has no TCP payload set; applying the %s preset %s", preset.label, formatPayload(&preset.payload))
		return &preset.payload
	default:
		logger.Debug("controller has no TCP payload set and no UFACTORY gripper is registered; leaving it unchanged")
		return nil
	}
}

func warnIfLighterThanPreset(what string, p *TCPPayloadConfig, preset *gripperPreset, logger logging.Logger) {
	if preset == nil || p.MassKg >= preset.payload.MassKg {
		return
	}
	logger.Warnf("%s mass %.3fkg is lighter than the %s preset's %.3fkg; the arm may sag or drift in manual mode. "+
		"Include the gripper and any tooling or held part in the payload.",
		what, p.MassKg, preset.label, preset.payload.MassKg)
}

func formatPayload(p *TCPPayloadConfig) string {
	if p == nil {
		return "unknown"
	}
	return fmt.Sprintf("mass=%.3fkg center_of_gravity=(%.1f, %.1f, %.1f)mm",
		p.MassKg, p.CenterOfGravityMM.X, p.CenterOfGravityMM.Y, p.CenterOfGravityMM.Z)
}

// payloadMatches reports whether the controller already holds p. The report
// carries float32 values, so exact comparison would always fail.
func payloadMatches(p, reported *TCPPayloadConfig) bool {
	const massTolKg, cogTolMM = 1e-3, 0.1
	if reported == nil {
		return false
	}
	d := p.CenterOfGravityMM.Sub(reported.CenterOfGravityMM)
	return math.Abs(p.MassKg-reported.MassKg) <= massTolKg &&
		math.Abs(d.X) <= cogTolMM && math.Abs(d.Y) <= cogTolMM && math.Abs(d.Z) <= cogTolMM
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

// ensureTCPPayload reads the controller's payload and sends whatever
// decideTCPPayload picks. Sending requires mode 0 / state 0 (see
// setTCPPayload); pass inPositionMode when the caller has already put the arm
// there, otherwise the mode is switched only if something is actually sent.
func (x *xArm) ensureTCPPayload(ctx context.Context, inPositionMode bool) error {
	var reported *TCPPayloadConfig
	if load, err := x.reportedTCPLoad(ctx); err != nil {
		x.logger.Warnf("could not read the controller's TCP payload: %v", err)
	} else {
		reported = &TCPPayloadConfig{MassKg: load.MassKg, CenterOfGravityMM: load.CenterOfGravityMM}
	}

	var configured *TCPPayloadConfig
	if x.conf != nil {
		configured = x.conf.TCPPayload
	}
	p := decideTCPPayload(configured, reported, x.gripperPreset.Load(), x.logger)

	if p != nil && !payloadMatches(p, reported) {
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
