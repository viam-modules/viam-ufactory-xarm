package arm

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
)

// defaultGripperPayloads is the payload a UFACTORY gripper puts on the flange.
// The xArm gripper and vacuum gripper values are UFACTORY Studio's built-in
// presets; the G2 shares the G1 preset there. UFACTORY publishes no preset for
// the Lite 6 grippers, so theirs come from the inertial blocks of the bundled
// URDFs.
var defaultGripperPayloads = map[string]TCPPayloadConfig{
	ModelNameGripper:           {MassKg: 0.82, CenterOfGravityMM: r3.Vector{Z: 48}},
	ModelNameVacuumGripper:     {MassKg: 0.61, CenterOfGravityMM: r3.Vector{Z: 53}},
	ModelNameGripperLite:       {MassKg: 0.25, CenterOfGravityMM: r3.Vector{Z: 30}},
	ModelNameVacuumGripperLite: {MassKg: 0.14, CenterOfGravityMM: r3.Vector{Z: 30}},
}

// effectiveTCPPayload is the payload to send to the controller: the arm's own
// tcp_payload when configured, otherwise the default of an attached gripper, or
// nil to leave the controller's current payload alone.
func (x *xArm) effectiveTCPPayload() *TCPPayloadConfig {
	if x.conf != nil && x.conf.TCPPayload != nil {
		return x.conf.TCPPayload
	}
	return x.gripperPayload.Load()
}

// claimGripperPayload installs a gripper model's default payload on the arm
// unless the arm configures its own. It returns a release func for the
// gripper's Close. The payload is sent lazily by the arm's next start() or
// manual-mode entry, so a gripper never switches controller modes underneath a
// running motion.
func claimGripperPayload(a arm.Arm, gripperModel string, logger logging.Logger) func() {
	noop := func() {}
	p, ok := defaultGripperPayloads[gripperModel]
	if !ok {
		return noop
	}
	x, ok := a.(*xArm)
	if !ok {
		logger.Debugf("arm is not a local xArm; not setting the %s default TCP payload", gripperModel)
		return noop
	}
	if x.conf != nil && x.conf.TCPPayload != nil {
		logger.Infof("arm %s configures tcp_payload; ignoring the %s default payload", x.name.Name, gripperModel)
		return noop
	}

	claimed := &p
	x.gripperPayload.Store(claimed)
	x.tcpPayloadApplied.Store(false)
	logger.Infof("using the %s default TCP payload on arm %s: mass=%.3fkg center_of_gravity=(%.1f, %.1f, %.1f)mm",
		gripperModel, x.name.Name, p.MassKg, p.CenterOfGravityMM.X, p.CenterOfGravityMM.Y, p.CenterOfGravityMM.Z)

	// Compare-and-swap so closing a gripper never clears a payload that a newer
	// gripper on the same arm has since installed. The controller keeps the last
	// payload it was sent; nothing is sent here.
	return func() { x.gripperPayload.CompareAndSwap(claimed, nil) }
}
