package arm

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"net"
	"sync"
	"time"

	"go.uber.org/multierr"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/ml"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	rutils "go.viam.com/rdk/utils"
	"go.viam.com/utils"
	"gorgonia.org/tensor"
)

const servoMotionMode = 1
const manualMode = 2
const errorState = 1 << 6
const warningState = 1 << 5
const notReadyForMotionState = 1 << 4
const ftSensorValueCount = 6

var regMap = map[string]byte{
	"Version":        0x01,
	"ActualCurrent":  0x05,
	"Shutdown":       0x0A,
	"ToggleServo":    0x0B,
	"SetState":       0x0C,
	"GetState":       0x0D,
	"CmdCount":       0x0E,
	"GetError":       0x0F,
	"ClearError":     0x10,
	"ClearWarn":      0x11,
	"SetMode":        0x13,
	"P2PJoint":       0x17,
	"MoveJoints":     0x1D,
	"ZeroJoints":     0x19,
	"JointPos":       0x2A,
	"Sensitivity":    0x25,
	"SetBound":       0x34,
	"EnableBound":    0x34,
	"CurrentTorque":  0x37,
	"FTSensorData":   0xC8,
	"FTSensorZero":   0xCE,
	"SetEEModel":     0x4E,
	"ServoError":     0x6A,
	"GripperControl": 0x7C,
	"VacuumControl":  0x7F,
	"LoadID":         0xCC,
	"VacuumState":    0x80,
}

const (
	ioControlParameterWord1High = 257
	ioControlParameterWord2High = 514
	ioControlParameterWord1Low  = 256
	ioControlParameterWord2Low  = 512
)

type cmd struct {
	tid    uint16
	prot   uint16
	reg    byte
	params []byte
}

func (c *cmd) bytes() []byte {
	var bin []byte
	uintBin := make([]byte, 2)
	binary.BigEndian.PutUint16(uintBin, c.tid)
	bin = append(bin, uintBin...)
	binary.BigEndian.PutUint16(uintBin, c.prot)
	bin = append(bin, uintBin...)
	binary.BigEndian.PutUint16(uintBin, 1+uint16(len(c.params))) //nolint:gosec
	bin = append(bin, uintBin...)
	bin = append(bin, c.reg)
	bin = append(bin, c.params...)
	return bin
}

// modbusConn encapsulates a single TCP connection to the xArm controller.
// Each instance owns its own lock and Transaction ID counter so that distinct
// instances (e.g. a separate gripper-bus connection on port 503) can run truly
// in parallel against the controller without contending on a shared mutex.
type modbusConn struct {
	addr    string // host:port
	logger  logging.Logger
	onReset func() // optional callback invoked after the socket is reset (used by cmdConn to clear x.started)

	lock sync.Mutex
	conn net.Conn
	tid  uint16
}

func newModbusConn(addr string, logger logging.Logger, onReset func()) *modbusConn {
	return &modbusConn{addr: addr, logger: logger, onReset: onReset}
}

func (m *modbusConn) connect(ctx context.Context) error {
	m.resetConnection()

	var d net.Dialer
	c, err := d.DialContext(ctx, "tcp", m.addr)
	if err != nil {
		return err
	}
	m.conn = c
	return nil
}

// close shuts down the socket. Safe to call multiple times.
func (m *modbusConn) close() error {
	if m.conn == nil {
		return nil
	}
	err := m.conn.Close()
	m.conn = nil
	return err
}

func (m *modbusConn) resetConnection() {
	if m.conn != nil {
		if err := m.conn.Close(); err != nil {
			m.logger.Infof("error closing old socket %s: %v", m.addr, err)
		}
		m.conn = nil
	}
	if m.onReset != nil {
		m.onReset()
	}
}

func (m *modbusConn) newCmd(reg byte) cmd {
	m.tid++
	return cmd{tid: m.tid, prot: 2, reg: reg}
}

func (m *modbusConn) send(ctx context.Context, c cmd, checkError bool) (cmd, error) {
	resp, err := m.writeBytes(ctx, c)
	if err != nil {
		return cmd{}, err
	}

	// check the error returned by the response
	if checkError {
		state := resp.params[0]
		// the 2nd and 3rd MSB in state byte indicate if there
		// is an error or warning respectively.
		if state&(errorState|warningState) != 0 {
			params, err := m.getErrorParams(ctx)
			if err != nil {
				return cmd{}, err
			}
			errCode := params[1]
			if errCode == errCodeCollision {
				// overcurrent estop has occurred, must be manually cleared by user.
				return cmd{}, fmt.Errorf("collision caused overcurrent: ensure robot is clear of obstacles and clear error " +
					"through UFACTORY Studio or clear_error do command")
			}
			// Check for manual mode error (0x25)
			if errCode == 0x25 {
				return cmd{}, fmt.Errorf("arm is in manual mode: use DoCommand with 'exit_manual_mode' to return to normal operation")
			}
			// Any other errors are cleared automatically by the driver.
			return cmd{}, decodeError(params)
		}
	}
	return resp, err
}

func (m *modbusConn) writeBytes(ctx context.Context, c cmd) (cmd, error) {
	m.lock.Lock()
	defer m.lock.Unlock()

	if m.conn == nil { // THIS HAS TO BE DONE INSIDE THE LOCK
		if err := m.connect(ctx); err != nil {
			m.resetConnection()
			return cmd{}, err
		}
	}

	b := c.bytes()
	// add deadline so we aren't waiting forever
	if err := m.conn.SetDeadline(time.Now().Add(5 * time.Second)); err != nil {
		m.resetConnection()
		return cmd{}, err
	}
	if _, err := m.conn.Write(b); err != nil {
		m.resetConnection()
		return cmd{}, err
	}
	return m.responseInLock(ctx)
}

func (m *modbusConn) responseInLock(ctx context.Context) (cmd, error) {
	buf, err := utils.ReadBytes(ctx, m.conn, 7)
	if err != nil {
		m.resetConnection()
		return cmd{}, err
	}
	c := cmd{}
	c.tid = binary.BigEndian.Uint16(buf[0:2])
	c.prot = binary.BigEndian.Uint16(buf[2:4])
	c.reg = buf[6]
	length := binary.BigEndian.Uint16(buf[4:6])
	c.params, err = utils.ReadBytes(ctx, m.conn, int(length-1))
	if err != nil {
		m.resetConnection()
		return cmd{}, err
	}
	return c, err
}

// getErrorParams queries the GetError register on this connection. Used by
// send() when the response state byte indicates a fault. Lives on modbusConn
// because the error-clear sequence must run on the same socket as the failed
// command.
func (m *modbusConn) getErrorParams(ctx context.Context) ([]byte, error) {
	c := m.newCmd(regMap["GetError"])
	resp, err := m.writeBytes(ctx, c)
	if err != nil {
		return nil, err
	}
	if len(resp.params) < 3 {
		return nil, errors.New("bad arm error query response")
	}
	return resp.params, nil
}

// Thin xArm-level wrappers that route to the command connection (port 502)
// by default. Most existing callers stay unchanged. Gripper-bus helpers
// (those using GripperControl/RS485_RTU) call x.gripperConn.send directly via
// gripperPreamble so they pick up the port 503 connection when it's
// available.

func (x *xArm) newCmd(reg byte) cmd {
	return x.cmdConn.newCmd(reg)
}

func (x *xArm) send(ctx context.Context, c cmd, checkError bool) (cmd, error) {
	if x.closed.Load() {
		return cmd{}, errors.New("closed")
	}
	return x.cmdConn.send(ctx, c, checkError)
}

func (x *xArm) getErrorParams(ctx context.Context) ([]byte, error) {
	return x.cmdConn.getErrorParams(ctx)
}

func (x *xArm) connect(ctx context.Context) error {
	return x.cmdConn.connect(ctx)
}

// checkServoErrors will query the individual servos for any servo-specific
// errors. It may be useful for troubleshooting but as the SDK does not call
// it directly ever, we probably don't need to either during normal operation.
func (x *xArm) CheckServoErrors(ctx context.Context) error {
	c := x.newCmd(regMap["ServoError"])
	e, err := x.send(ctx, c, false)
	if err != nil {
		return err
	}
	if len(e.params) < 18 {
		return errors.New("bad servo error query response")
	}

	// Get error codes for all (8) servos.
	// xArm 6 has 6, xArm 7 has 7, and plus one in the xArm gripper
	for i := 1; i < 9; i++ {
		errCode := e.params[i*2]
		errMsg, isErr := servoErrorMap[errCode]
		if isErr {
			err = multierr.Append(err, errors.New(errMsg))
		}
	}
	return err
}
func (x *xArm) checkReadyState(ctx context.Context, enableMotion bool) error {
	// read the current arm state
	// bit6 is 1 if there is an error
	// bit5 is 1 if there is a warning
	// bit4 is 1 if not ready for motion
	currentState, err := x.getErrorParams(ctx)
	if err != nil {
		// most likely indicative of a communication issue
		return multierr.Combine(fmt.Errorf("cannot check the state of the arm, check communication configuration"), err)
	}

	if currentState[0]&errorState != 0 {
		// we assume that if we run into an error we will need to restart the servos etc.
		x.started.Store(-1)

		// we are in error state, we will attempt to clear the error
		// if we fail we will return the error code
		c := x.newCmd(regMap["ClearError"])
		newState, err := x.send(ctx, c, false)
		if err != nil {
			return multierr.Combine(fmt.Errorf("unable to reset the error %w", errors.New(armBoxErrorMap[currentState[1]])), err)
		}
		if newState.params[0]&errorState != 0 {
			return fmt.Errorf("the arm is in an error state and couldn't be reset, check if the e-stopped is released. Error is %w ",
				errors.New(armBoxErrorMap[currentState[1]]))
		}
		x.logger.Debugf("arm error %s has been cleared", errors.New(armBoxErrorMap[currentState[1]]))
	}
	if currentState[0]&warningState != 0 {
		// we are in warning state, we will attempt to clear the warning
		// if we fail we will return the warning code
		c := x.newCmd(regMap["ClearWarn"])
		newState, err := x.send(ctx, c, false)
		if err != nil {
			return multierr.Combine(fmt.Errorf("unable to reset the warning %w", errors.New(armBoxWarnMap[currentState[2]])), err)
		}
		if newState.params[0]&warningState != 0 {
			return fmt.Errorf("the arm is in an error state and couldn't be reset, check if the e-stopped is released. Error is %w ",
				errors.New(armBoxWarnMap[currentState[2]]))
		}
		x.logger.Debugf("arm error %s has been cleared", errors.New(armBoxWarnMap[currentState[2]]))
	}
	if currentState[0]&notReadyForMotionState != 0 && enableMotion {
		// Check if we're intentionally in manual mode - if so, don't "fix" it
		if x.started.Load() == int32(manualMode) {
			x.logger.Debug("Arm is in manual mode, skipping motion ready check")
			return nil
		}
		x.logger.Error("motion not ready will enable it")
		return multierr.Combine(x.setMotionMode(ctx, servoMotionMode), x.setMotionState(ctx, 0))
	}
	return nil
}

// setMotionState sets the motion state of the arm.
// Useful states:
// 0: Servo motion mode
// 3: Suspend current movement
// 4: Stop all motion, restart system
func (x *xArm) setMotionState(ctx context.Context, state byte) error {
	c := x.newCmd(regMap["SetState"])
	c.params = append(c.params, state)
	_, err := x.send(ctx, c, true)
	return err
}

// setMotionMode sets the motion mode of the arm.
// 0: Position Control Mode, i.e. "normal" mode
// 1: Servoj mode. This mode will immediately execute joint positions at the fastest available speed and is intended
// for streaming large numbers of joint positions to the arm.
// 2: Joint teaching mode, not useful right now
func (x *xArm) setMotionMode(ctx context.Context, state byte) error {
	c := x.newCmd(regMap["SetMode"])
	c.params = append(c.params, state)
	_, err := x.send(ctx, c, true)
	return err
}

// toggleServos toggles the servos on or off.
// True enables servos and disengages brakes.
// False disables servos without engaging brakes.
func (x *xArm) toggleServos(ctx context.Context, enable bool) error {
	c := x.newCmd(regMap["ToggleServo"])
	var enByte byte
	if enable {
		enByte = 1
	}
	c.params = append(c.params, 8, enByte)
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) start(ctx context.Context, direct bool) error {
	mode := byte(servoMotionMode)
	if direct {
		mode = 0
	}

	if x.started.Load() == int32(mode) {
		return nil
	}

	if err := x.checkReadyState(ctx, false); err != nil {
		return err
	}

	err := x.toggleServos(ctx, true)
	if err != nil {
		return err
	}

	err = x.setMotionMode(ctx, mode)
	if err != nil {
		return err
	}
	if err := x.setMotionState(ctx, 0); err != nil {
		return err
	}
	x.started.Store(int32(mode))
	return nil
}

// enterManualMode puts the arm into manual mode (joint teaching mode).
// In mode 2, the arm enters zero gravity mode with automatic gravity compensation,
// allowing free movement by hand. Servos remain active to provide gravity
// compensation torque - do NOT disable them or the arm will sag.
func (x *xArm) enterManualMode(ctx context.Context) error {
	x.logger.Info("Entering manual mode (zero gravity mode)")

	// Clear any errors before entering manual mode
	if err := x.checkReadyState(ctx, false); err != nil {
		x.logger.Warnf("Could not fully clear ready state: %v", err)
	}

	// It appears that the xArm controller cannot transition from mode 1 (servoJ)
	// to mode 2 (teach mode)
	// As a work around we transition the controller to mode 0 (position) then
	// to teach mode preventing having to call enterManualMode twice
	if err := x.setMotionMode(ctx, 0); err != nil {
		return fmt.Errorf("failed to set position mode before manual: %w", err)
	}
	if err := x.setMotionState(ctx, 0); err != nil {
		return fmt.Errorf("failed to activate position mode before manual: %w", err)
	}

	// Set motion mode to 2 (manual/teaching mode)
	// The firmware keeps servos active with only gravity-compensating torque,
	// so the arm holds position but can be freely moved by hand.
	if err := x.setMotionMode(ctx, manualMode); err != nil {
		return fmt.Errorf("failed to set manual mode: %w", err)
	}

	// Set motion state to 0 to activate the mode
	if err := x.setMotionState(ctx, 0); err != nil {
		return fmt.Errorf("failed to activate manual mode: %w", err)
	}

	x.started.Store(int32(manualMode))
	x.logger.Info("Manual mode enabled - arm is freely moveable by hand with gravity compensation")

	return nil
}

// exitManualMode exits manual mode and returns the arm to normal operation.
func (x *xArm) exitManualMode(ctx context.Context) error {
	x.logger.Info("Exiting manual mode")

	// Reset internal state flag to force re-initialization
	x.started.Store(-1)

	// Use the existing start function to return to normal servo mode (mode 1)
	// This will:
	// - Set mode back to servoMotionMode (1)
	// - Enable servos properly
	// - Set state to 0 (ready)
	if err := x.start(ctx, false); err != nil {
		return fmt.Errorf("failed to exit manual mode: %w", err)
	}

	x.logger.Info("Manual mode exited - arm ready for programmatic commands")
	return nil
}

// Close shuts down the arm servos and engages brakes.
func (x *xArm) Close(ctx context.Context) error {
	if x.proxyServer != nil {
		x.stopProxy()
	}

	if x.cmdConn == nil || x.cmdConn.conn == nil {
		x.closed.Store(true)
		return nil
	}

	stopErr := x.setMotionState(ctx, 3)
	closeErr := x.cmdConn.close()
	var gripperCloseErr error
	if x.gripperConn != nil && x.gripperConn != x.cmdConn {
		gripperCloseErr = x.gripperConn.close()
	}
	err := multierr.Combine(stopErr, closeErr, gripperCloseErr)

	if err != nil {
		x.logger.Warnf("closing connection failed: %v", err)
	}

	x.closed.Store(true)

	return err
}

func (x *xArm) MoveThroughJointPositions(
	ctx context.Context,
	positions [][]referenceframe.Input,
	opts *arm.MoveOptions,
	extra map[string]any,
) error {
	mo := x.moveOptions(opts, extra)
	return x.internalMoveThroughJointPositions(ctx, positions, mo)
}

func (x *xArm) internalMoveThroughJointPositions(
	ctx context.Context,
	positions [][]referenceframe.Input,
	mo moveOptions,
) error {
	ctx, done := x.opMgr.New(ctx)
	defer done()

	if mo.direct && len(positions) > 1 {
		return fmt.Errorf("direct only work with 1 position, send %d", len(positions))
	}

	if err := x.checkReadyState(ctx, true); err != nil {
		return err
	}

	for _, goal := range positions {
		// check that joint positions are not out of bounds
		if err := arm.CheckDesiredJointPositions(ctx, x, goal); err != nil {
			return err
		}
	}

	armRawSteps := positions
	if x.trajGen != nil {
		curPos, err := x.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		trajSteps, err := x.createTrajGenSteps(ctx, curPos, positions)
		if err != nil {
			return err
		}
		// trajectory generation thinks we are already at our goal, so don't move.
		if trajSteps == nil {
			return nil
		}
		armRawSteps = trajSteps
	} else if !mo.direct && mo.interpolate {
		curPos, err := x.JointPositions(ctx, nil)
		if err != nil {
			return err
		}
		armRawSteps, err = x.createRawJointSteps(curPos, positions, mo)
		if err != nil {
			return err
		}
	}

	return x.executeInputs(ctx, armRawSteps, mo)
}

func (x *xArm) createTrajGenSteps(
	ctx context.Context,
	curPos []referenceframe.Input,
	positions [][]referenceframe.Input,
) ([][]referenceframe.Input, error) {
	nWaypoints := len(positions) + 1
	waypoints := make([]float64, 0, nWaypoints*x.dof)
	for _, inp := range curPos {
		waypoints = append(waypoints, inp)
	}
	for _, wp := range positions {
		for _, inp := range wp {
			waypoints = append(waypoints, inp)
		}
	}

	x.confLock.Lock()
	speed := x.speed
	accel := x.acceleration
	x.confLock.Unlock()

	velLimits := make([]float64, x.dof)
	accelLimits := make([]float64, x.dof)
	for i := range velLimits {
		velLimits[i] = speed
		accelLimits[i] = accel
	}

	x.logger.Debugf("calling trajectory generator with %d waypoints", nWaypoints)
	outMap, err := x.trajGen.Infer(ctx, ml.Tensors{
		"waypoints_rads": tensor.New(
			tensor.Of(tensor.Float64),
			tensor.WithShape(nWaypoints, x.dof),
			tensor.WithBacking(waypoints),
		),
		"velocity_limits_rads_per_sec": tensor.New(
			tensor.Of(tensor.Float64),
			tensor.WithShape(x.dof),
			tensor.WithBacking(velLimits),
		),
		"acceleration_limits_rads_per_sec2": tensor.New(
			tensor.Of(tensor.Float64),
			tensor.WithShape(x.dof),
			tensor.WithBacking(accelLimits),
		),
		"path_tolerance_delta_rads": tensor.New(
			tensor.Of(tensor.Float64),
			tensor.WithShape(1),
			tensor.WithBacking([]float64{*x.conf.TrajGen.PathToleranceDeltaRads}),
		),
		"path_colinearization_ratio": tensor.New(
			tensor.Of(tensor.Float64),
			tensor.WithShape(1),
			tensor.WithBacking([]float64{*x.conf.TrajGen.PathColinearizationRatio}),
		),
		"waypoint_deduplication_tolerance_rads": tensor.New(
			tensor.Of(tensor.Float64),
			tensor.WithShape(1),
			tensor.WithBacking([]float64{*x.conf.TrajGen.WaypointDeduplicationToleranceRads}),
		),
		"trajectory_sampling_freq_hz": tensor.New(
			tensor.Of(tensor.Int64),
			tensor.WithShape(1),
			tensor.WithBacking([]int64{int64(x.moveHZ)}),
		),
	})
	if err != nil {
		return nil, err
	}

	configsTensor, ok := outMap["configurations_rads"]
	if !ok {
		// Service returns an empty map when fewer than 2 distinct waypoints
		// remain after deduplication -- the arm is already at the goal.
		return nil, nil
	}
	configsData := configsTensor.Data().([]float64)
	nSamples := configsTensor.Shape()[0]
	x.logger.Debugf("trajectory generator produced %d samples", nSamples)
	steps := make([][]referenceframe.Input, nSamples)
	for i := range nSamples {
		step := make([]referenceframe.Input, x.dof)
		for j := range x.dof {
			step[j] = configsData[i*x.dof+j]
		}
		steps[i] = step
	}
	return steps, nil
}

func (x *xArm) clampMoveOptions(val, minVal, maxVal float64, name string) float64 {
	if val == 0 {
		return val
	}
	if val < minVal {
		x.logger.Warnf("invalid %s option %.2f: setting to minimum %.2f", name, val, minVal)
		return minVal
	}
	if val > maxVal {
		x.logger.Warnf("invalid %s option %.2f: setting to maximum %.2f", name, val, maxVal)
		return maxVal
	}
	return val
}

// Using the configured moveHz, joint speed, and joint acceleration, create the series of joint positions for the arm to follow,
// using a trapezoidal velocity profile to blend between waypoints to the extent possible.
func (x *xArm) createRawJointSteps(
	startInputs []referenceframe.Input,
	inputSteps [][]referenceframe.Input,
	mo moveOptions,
) (
	[][]float64, error) {
	// Generate list of joint positions to pass through
	// This is almost-calculus but not quite because it's explicitly discretized
	accelStep := mo.acceleration / mo.moveHZ
	interwaypointAccelStep := interwaypointAccel / mo.moveHZ

	from := startInputs

	// We want smooth acceleration/motion but there's no guarantee the provided inputs have continuous velocity signs
	floatMaxDiff := func(from, to []float64) float64 {
		maxVal := 0.
		for i, toInput := range to {
			diff := math.Abs(toInput - from[i])
			if diff > maxVal {
				maxVal = diff
			}
		}
		return maxVal
	}

	// Preprocess steps into step counts
	stepTotal := 0.
	displacementTotal := 0.

	for _, toInputs := range inputSteps {
		maxVal := floatMaxDiff(from, toInputs)
		displacementTotal += maxVal
		nSteps := (math.Abs(maxVal) / mo.speed) * mo.moveHZ
		stepTotal += nSteps
		from = toInputs
	}

	nominalAccelSteps := (mo.speed / mo.acceleration) * mo.moveHZ // This many steps to accelerate, and the same to decelerate
	if nominalAccelSteps > stepTotal*0.95 {
		nominalAccelSteps = 0.95 * math.Sqrt(displacementTotal/mo.acceleration) * mo.moveHZ
	}
	maxVel := (nominalAccelSteps / mo.moveHZ) * mo.acceleration

	inputStepsReversed := [][]referenceframe.Input{}
	for i := len(inputSteps) - 1; i >= 0; i-- {
		inputStepsReversed = append(inputStepsReversed, inputSteps[i])
	}
	inputStepsReversed = append(inputStepsReversed, startInputs)

	accelCurve := func(
		startInputs []referenceframe.Input,
		allInputSteps [][]referenceframe.Input,
		stopVel float64,
	) (int, [][]float64, error) {
		currSpeed := math.Min(accelStep, mo.speed)
		steps := [][]float64{}
		from = startInputs
		lastInputs := startInputs
		for i, toInputs := range allInputSteps {
			runningFrom := from

			for currDiff := floatMaxDiff(runningFrom, toInputs); currDiff > 1e-6; currDiff = floatMaxDiff(runningFrom, toInputs) {
				if currSpeed <= 0 {
					break
				}
				nSteps := (currDiff / currSpeed) * mo.moveHZ
				stepSize := 1.
				if nSteps <= 1 {
					if currDiff == 0 {
						break
					}
					stepSize = nSteps
				}
				nextInputs, err := x.model.Interpolate(lastInputs, toInputs, stepSize/nSteps)
				if err != nil {
					return 0, nil, err
				}
				runningFrom = nextInputs
				steps = append(steps, nextInputs)

				if currSpeed < mo.speed {
					currSpeed += accelStep * stepSize
					if currSpeed > mo.speed {
						currSpeed = mo.speed
					}
				} else {
					// If we reach max speed, accelerate at max for the remainder of the move
					accelStep = interwaypointAccelStep
				}

				if currSpeed >= stopVel-1e-6 {
					return i, steps, nil
				}

				lastInputs = nextInputs
			}
			lastInputs = toInputs
			from = toInputs
		}
		return len(allInputSteps), steps, nil
	}

	decelStart, decelSteps, err := accelCurve(inputStepsReversed[0], inputStepsReversed, maxVel)
	if err != nil {
		return nil, err
	}
	accelStop := len(inputSteps) - decelStart
	accelInputSteps := [][]referenceframe.Input{}
	for i, inputStep := range inputSteps {
		if i == accelStop {
			accelInputSteps = append(accelInputSteps, decelSteps[len(decelSteps)-1])
			break
		}
		accelInputSteps = append(accelInputSteps, inputStep)
	}
	_, accelSteps, err := accelCurve(startInputs, accelInputSteps, math.Inf(1))
	if err != nil {
		return nil, err
	}
	for i := len(decelSteps) - 2; i >= 0; i-- {
		accelSteps = append(accelSteps, decelSteps[i])
	}

	return accelSteps, nil
}

func (x *xArm) executeInputs(ctx context.Context, rawSteps [][]float64, mo moveOptions) error {
	if err := x.start(ctx, mo.direct); err != nil {
		return err
	}
	// convenience for structuring and sending individual joint steps
	for stepIdx, step := range rawSteps {
		loopTimeStart := time.Now()

		cName := "MoveJoints"
		if mo.direct {
			cName = "P2PJoint"
		}
		c := x.newCmd(regMap[cName])
		jFloatBytes := make([]byte, 4)
		for _, jRad := range step {
			binary.LittleEndian.PutUint32(jFloatBytes, math.Float32bits(float32(jRad)))
			c.params = append(c.params, jFloatBytes...)
		}
		// xarm 6 has 6 joints, but protocol needs 7- add 4 bytes for a blank 7th joint
		for dof := x.dof; dof < 7; dof++ {
			c.params = append(c.params, 0, 0, 0, 0)
		}

		// speed
		binary.LittleEndian.PutUint32(jFloatBytes, math.Float32bits(float32(mo.speed)))
		c.params = append(c.params, jFloatBytes...)
		// acceleration
		binary.LittleEndian.PutUint32(jFloatBytes, math.Float32bits(float32(mo.acceleration)))
		c.params = append(c.params, jFloatBytes...)
		// Motion Time - not used by the arm yet
		c.params = append(c.params, 0, 0, 0, 0)

		_, err := x.send(ctx, c, true)
		if err != nil {
			return err
		}

		if mo.waitAtEnd || (stepIdx+1) < len(rawSteps) {
			sleepTime := (time.Duration(1000000./mo.moveHZ) * time.Microsecond) - time.Since(loopTimeStart)
			if sleepTime < 0 {
				x.logger.Warnf("sleepTime is negative (%v) which means we aren't sending joints fast enough", sleepTime)
			}
			// `MoveJoints` API calls are async. The response is immediate. We guess how long to sleep
			// before issuing the next `MoveJoints` command.
			if !utils.SelectContextOrWait(ctx, sleepTime) {
				return ctx.Err()
			}
		}
	}

	// Our guessing for how long to wait usually accumulates to be 10% off by the end. Wait until
	// the arm has definitely stopped moving by polling the arm state.
	for mo.waitAtEnd && ctx.Err() == nil {
		stateCmd := x.newCmd(regMap["GetState"])
		resp, err := x.send(ctx, stateCmd, true)
		if err != nil {
			return fmt.Errorf("error getting state waiting for movement to stop: %w", err)
		}

		if resp.params[0] == 0x00 && resp.params[1] == 0x01 {
			// Still moving.
			time.Sleep(10 * time.Millisecond)
		} else {
			break
		}
	}

	return ctx.Err()
}

// EndPosition computes and returns the current cartesian position.
func (x *xArm) EndPosition(ctx context.Context, extra map[string]any) (spatialmath.Pose, error) {
	joints, err := x.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return x.model.Transform(joints)
}

// MoveToPosition moves the arm to the specified cartesian position.
func (x *xArm) MoveToPosition(ctx context.Context, pos spatialmath.Pose, extra map[string]any) error {
	if x.motion == nil {
		return fmt.Errorf("xarm cannot do MoveToPosition without speficying a motion service")
	}

	_, err := x.motion.Move(
		ctx,
		motion.MoveReq{
			ComponentName: x.Name().Name,
			Destination:   referenceframe.NewPoseInFrame(fmt.Sprintf("%v_origin", x.Name().Name), pos),
		},
	)
	return err
}

// JointPositions returns the current positions of all joints.
func (x *xArm) JointPositions(ctx context.Context, extra map[string]any) ([]referenceframe.Input, error) {
	if err := x.checkReadyState(ctx, false); err != nil {
		return nil, err
	}

	c := x.newCmd(regMap["JointPos"])

	jData, err := x.send(ctx, c, true)
	if err != nil {
		return nil, err
	}
	var radians []float64

	if jData.params == nil {
		return nil, errors.New("couldn't get joint positions")
	}
	// didn't return expected bytes
	if len(jData.params) < x.dof*4 {
		return nil, fmt.Errorf("unexpected return getting joint positions, got %d want %d", len(jData.params), x.dof)
	}
	for i := 0; i < x.dof; i++ {
		idx := i*4 + 1
		radians = append(radians, float64(rutils.Float32FromBytesLE((jData.params[idx : idx+4]))))
	}
	return radians, nil
}

// Stop stops the xArm but also reinitializes the arm so it can take commands again.
func (x *xArm) Stop(ctx context.Context, extra map[string]any) error {
	ctx, done := x.opMgr.New(ctx)
	defer done()

	x.started.Store(-1)

	if err := x.setMotionState(ctx, 3); err != nil {
		return err
	}

	return x.start(ctx, false)
}

// IsMoving returns whether the arm is moving.
func (x *xArm) IsMoving(ctx context.Context) (bool, error) {
	return x.opMgr.OpRunning(), nil
}

func (x *xArm) setupGripper(ctx context.Context) error {
	if err := x.disableGripperControlMode(ctx); err != nil {
		return err
	}
	if err := x.enableGripper(ctx); err != nil {
		return err
	}
	if err := x.setGripperMode(ctx, false); err != nil {
		return err
	}
	return nil
}

// disableGripperControlMode clears the FnC00 "Control Enable" register (0x0C00) set by graspWithTorque.
// FnC00 enables the FnCxx block-write control mode (speed+torque+position); when left enabled,
// subsequent standalone Fn700 position commands may not work reliably. See G2 manual section 4.1.7.
func (x *xArm) disableGripperControlMode(ctx context.Context) error {
	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x0C, 0x00)
	c.params = append(c.params, 0x00, 0x01)
	c.params = append(c.params, 0x02)
	c.params = append(c.params, 0x00, 0x00)
	x.logger.Debugf("disableGripperControlMode")
	_, err := x.send(ctx, c, true)
	return err
}

// gripperPreamble routes through gripperConn so that gripper-bus traffic
// uses the dedicated port-503 socket when available. If port 503 wasn't
// reachable at boot, gripperConn aliases cmdConn and behavior collapses to
// the shared-socket case.
func (x *xArm) gripperPreamble(write bool) cmd {
	c := x.gripperConn.newCmd(regMap["GripperControl"])
	c.params = append(c.params, 0x09) // host id
	c.params = append(c.params, 0x08) // gripper id
	if write {
		c.params = append(c.params, 0x10)
	} else {
		c.params = append(c.params, 0x03)
	}
	return c
}

// gripperSend mirrors x.send but routes through gripperConn. checkError is
// always true on this path — gripper Modbus failures should surface, never
// be swallowed silently.
func (x *xArm) gripperSend(ctx context.Context, c cmd) (cmd, error) {
	if x.closed.Load() {
		return cmd{}, errors.New("closed")
	}
	return x.gripperConn.send(ctx, c, true)
}

func (x *xArm) enableGripper(ctx context.Context) error {
	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x01, 0x00)
	c.params = append(c.params, 0x00, 0x01)
	c.params = append(c.params, 0x02)
	c.params = append(c.params, 0x00, 0x01)
	_, err := x.gripperSend(ctx, c)
	return err
}

func (x *xArm) setGripperMode(ctx context.Context, speed bool) error {
	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x01, 0x01)
	c.params = append(c.params, 0x00, 0x01)
	c.params = append(c.params, 0x02)
	if speed {
		c.params = append(c.params, 0x00, 0x01)
	} else {
		c.params = append(c.params, 0x00, 0x00)
	}
	_, err := x.gripperSend(ctx, c)
	return err
}

func (x *xArm) setGripperPosition(ctx context.Context, position uint32) error {
	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x07, 0x00)
	c.params = append(c.params, 0x00, 0x02)
	c.params = append(c.params, 0x04)
	tmpBytes := make([]byte, 4)
	binary.BigEndian.PutUint32(tmpBytes, position)
	x.logger.Debugf("setGripperPosition bytes: %v", tmpBytes)
	c.params = append(c.params, tmpBytes...)
	_, err := x.gripperSend(ctx, c)
	return err
}

func (x *xArm) setGripperSpeed(ctx context.Context, speed uint16) error {
	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x03, 0x03)
	c.params = append(c.params, 0x00, 0x01)
	c.params = append(c.params, 0x02)
	tmpBytes := make([]byte, 2)
	binary.BigEndian.PutUint16(tmpBytes, speed)
	x.logger.Debugf("setGripperSpeed bytes: %v", tmpBytes)
	c.params = append(c.params, tmpBytes...)
	_, err := x.gripperSend(ctx, c)
	return err
}

func (x *xArm) getGripperSpeed(ctx context.Context) (uint16, error) {
	c := x.gripperPreamble(false)
	c.params = append(c.params, 0x03, 0x03)
	c.params = append(c.params, 0x00, 0x01)
	res, err := x.gripperSend(ctx, c)
	if err != nil {
		return 0, err
	}

	x.logger.Debugf("getGripperSpeed: %v %v", res, res.params)

	if len(res.params) != 7 {
		return 0, fmt.Errorf("unexpected length for getGripperSpeed response: %d %v", len(res.params), res.params)
	}

	return binary.BigEndian.Uint16(res.params[5:]), nil
}

// graspWithTorque issues the FnCxx block-write (start address 0x0C00, 5 registers) so the gripper
// applies the requested grasp current/torque atomically with the position move. See section 4.2 of
// the G2 manual — this mirrors the Python SDK's set_gripper_g2_position(position, speed, force).
func (x *xArm) graspWithTorque(ctx context.Context, speed, torque uint16, position uint32, stall time.Duration) error {
	// Clear FnC00 first so the firmware sees a 0->1 transition on the new write,
	// otherwise back-to-back grasp commands may be ignored while a hold is active.
	if err := x.disableGripperControlMode(ctx); err != nil {
		return err
	}

	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x0C, 0x00)
	c.params = append(c.params, 0x00, 0x05)
	c.params = append(c.params, 0x0A)

	buf := make([]byte, 2)
	binary.BigEndian.PutUint16(buf, 1) // FnC00 enable
	c.params = append(c.params, buf...)
	binary.BigEndian.PutUint16(buf, speed) // FnC01
	c.params = append(c.params, buf...)
	binary.BigEndian.PutUint16(buf, torque) // FnC02
	c.params = append(c.params, buf...)

	posBytes := make([]byte, 4)
	binary.BigEndian.PutUint32(posBytes, position) // FnC03 high, FnC04 low
	c.params = append(c.params, posBytes...)

	x.logger.Debugf("graspWithTorque speed=%d torque=%d position=%d stall=%s", speed, torque, position, stall)
	if _, err := x.send(ctx, c, true); err != nil {
		return err
	}

	return x.waitForGripper(ctx, int(position), stall) //nolint:gosec
}

// waitForGripper polls gripper position until it reaches goal (within 6),
// stalls (no movement >1 for >stall), or 10s elapses as an overall backstop.
// Caller controls the stall window: a long stall (e.g. 1s) confirms the gripper
// really stopped; a short stall (e.g. 200ms) returns quickly for callers that
// expect immediate resistance like squeeze loops.
func (x *xArm) waitForGripper(ctx context.Context, goal int, stall time.Duration) error {
	const pollInterval = 30
	const overallTimeout = 10 * time.Second
	stallMs := int(stall / time.Millisecond)
	start := time.Now()
	old := -1
	msSinceStuck := -1

	for {
		time.Sleep(time.Duration(pollInterval) * time.Millisecond)

		pos32, err := x.getGripperPosition(ctx)
		if err != nil {
			return err
		}
		pos := int(pos32)

		if math.Abs(float64(pos-goal)) <= 6 {
			return nil
		}

		if old >= 0 && math.Abs(float64(pos-old)) <= 1 {
			msSinceStuck += pollInterval
			if msSinceStuck > stallMs {
				return nil
			}
		} else {
			msSinceStuck = 0
		}
		old = pos

		if time.Since(start) > overallTimeout {
			return nil
		}
	}
}

func (x *xArm) getGripperPosition(ctx context.Context) (int32, error) {
	c := x.gripperPreamble(false)
	c.params = append(c.params, 0x07, 0x02)
	c.params = append(c.params, 0x00, 0x02)
	res, err := x.gripperSend(ctx, c)
	if err != nil {
		return 0, err
	}

	x.logger.Debugf("getGripperPosition: %v %v", res, res.params)

	// open  : 0 9 8 3 4 0 0 3 73
	// closed: 0 9 8 3 4 0 0 0 0
	if len(res.params) != 9 {
		return 0, fmt.Errorf("weird length for getGripperPosition response: %d %v", len(res.params), res.params)
	}

	return int32(binary.BigEndian.Uint32(res.params[5:])), nil //nolint:gosec
}

// vacuumStateFromResponse decodes "object picked" from a DIGITAL_IN response.
// v1 reads input pin 0 (bit 0x01); v2 reads input pin 3 (bit 0x04).
func vacuumStateFromResponse(params []byte, ct connectionType) (bool, error) {
	if len(params) != 5 {
		return false, fmt.Errorf("weird length for getVacuumStatus response: %d %v", len(params), params)
	}
	mask := byte(0x01)
	if ct == connectionContact {
		mask = 0x04
	}
	return params[4]&mask != 0, nil
}

func (x *xArm) getVacuumStatus(ctx context.Context, ct connectionType) (bool, error) {
	c := x.newCmd(regMap["VacuumState"])
	c.params = append(c.params, 0x09, 0x0A, 0x14)
	res, err := x.send(ctx, c, true)
	if err != nil {
		return false, err
	}
	return vacuumStateFromResponse(res.params, ct)
}

// This is the host ID and gripper address which should be appended to each command.
func (x *xArm) vacuumPreamble() cmd {
	c := x.newCmd(regMap["VacuumControl"])

	c.params = append(c.params, 0x09) // host ID
	c.params = append(c.params, 0x0A) // vacuum ID
	c.params = append(c.params, 0x15)
	return c
}

// tgpioCoreBits mirrors the xArm SDK tgpio_set_digital core-pin bit table.
// Keyed by core pin (userPin+1).
var tgpioCoreBits = map[int]struct{ mask, val uint16 }{
	1: {0x0100, 0x0001},
	2: {0x0200, 0x0002},
	3: {0x1000, 0x0010},
	4: {0x0400, 0x0004},
	5: {0x0800, 0x0008},
}

// tgpioWord builds the 16-bit mask|value word for a TGPIO digital-out write.
func tgpioWord(userPin int, value bool) uint16 {
	b := tgpioCoreBits[userPin+1]
	w := b.mask
	if value {
		w |= b.val
	}
	return w
}

// tgpioDigitalParams builds the params appended after the VacuumControl register
// byte: [bid=0x09][addr 0x0A,0x15][LE-fp32(word)].
func tgpioDigitalParams(userPin int, value bool) []byte {
	arg := make([]byte, 4)
	binary.LittleEndian.PutUint32(arg, math.Float32bits(float32(tgpioWord(userPin, value))))
	return append([]byte{0x09, 0x0A, 0x15}, arg...)
}

// sendTgpioDigital drives one TGPIO digital output (user pin) high/low.
func (x *xArm) sendTgpioDigital(ctx context.Context, userPin int, value bool) error {
	c := x.newCmd(regMap["VacuumControl"])
	c.params = append(c.params, tgpioDigitalParams(userPin, value)...)
	_, err := x.send(ctx, c, true)
	return err
}

// setVacuum drives the two vacuum pins: ON = pins[0] high & pins[1] low; OFF inverts.
func (x *xArm) setVacuum(ctx context.Context, ct connectionType, on bool) error {
	pins := vacuumPins(ct)
	if err := x.sendTgpioDigital(ctx, pins[0], on); err != nil {
		return err
	}
	return x.sendTgpioDigital(ctx, pins[1], !on)
}

// Grab maps to open in ufactory.
func (x *xArm) grabVacuum(ctx context.Context, ct connectionType) error {
	return x.setVacuum(ctx, ct, true)
}

func (x *xArm) makeIoControlParamenterCmd(word float32) cmd {
	arg := make([]byte, 4)
	binary.LittleEndian.PutUint32(arg, math.Float32bits(word))
	c := x.vacuumPreamble()
	c.params = append(c.params, arg...)
	return c
}

func (x *xArm) liteGripperAction(ctx context.Context, action string) (map[string]any, error) {
	// we use register 0x7F to control Robot Digital IO to open or close the lite gripper
	var err error
	switch action {
	case gripperLiteActionClose:
		if _, err = x.send(ctx, x.makeIoControlParamenterCmd(ioControlParameterWord2High), true); err != nil {
			return nil, err
		}
		if _, err = x.send(ctx, x.makeIoControlParamenterCmd(ioControlParameterWord1Low), true); err != nil {
			return nil, err
		}
		return map[string]any{}, nil
	case gripperLiteActionOpen:
		if _, err = x.send(ctx, x.makeIoControlParamenterCmd(ioControlParameterWord2Low), true); err != nil {
			return nil, err
		}

		if _, err = x.send(ctx, x.makeIoControlParamenterCmd(ioControlParameterWord1High), true); err != nil {
			return nil, err
		}
		return map[string]any{}, nil
	case gripperLiteActionStop:
		if _, err = x.send(ctx, x.makeIoControlParamenterCmd(ioControlParameterWord1Low), true); err != nil {
			return nil, err
		}
		if _, err = x.send(ctx, x.makeIoControlParamenterCmd(ioControlParameterWord2Low), true); err != nil {
			return nil, err
		}
		return map[string]any{}, nil
	case gripperLiteActionIsClosed:
		c := x.newCmd(regMap["VacuumState"])
		additionalParams := []byte{
			0x09,
			0x0A,
			0x18,
		}
		c.params = append(c.params, additionalParams...)
		res, err := x.send(ctx, c, true)
		if err != nil {
			return nil, err
		}
		if len(res.params) != 5 {
			return nil, fmt.Errorf("status register at address 0x18 returned an array of length %d expected length 5 raw data %v", len(res.params), res.params)
		}
		isHolding := false
		// byte 5 of register 0x18 is 0 when stopped, 1 when opened and 2 when closed
		if res.params[4] == 2 {
			isHolding = true
		}
		return map[string]any{gripperLiteActionIsClosed: isHolding}, nil
	}

	return nil, fmt.Errorf("gripper lite action %s is not supported", action)
}

// Close maps to open in ufactory.
func (x *xArm) openVacuum(ctx context.Context, ct connectionType) error {
	return x.setVacuum(ctx, ct, false)
}

func (x *xArm) getLoad(ctx context.Context) ([]float64, error) {
	c := x.newCmd(regMap["CurrentTorque"])
	// ~ c.params = append(c.params, 0x01)
	loadData, err := x.send(ctx, c, true)
	if err != nil {
		return []float64{}, err
	}
	var loads []float64
	for i := 0; i < x.dof; i++ {
		idx := i*4 + 1
		loads = append(loads, float64(rutils.Float32FromBytesLE((loadData.params[idx : idx+4]))))
	}

	return loads, nil
}

// parseFTSensorData parses FTSensorData (0xC8): params[0] is a status byte, then six
// little-endian float32 values at offset i*4+1.
func parseFTSensorData(params []byte) ([]float64, error) {
	need := 1 + ftSensorValueCount*4
	if len(params) < need {
		return nil, fmt.Errorf("unexpected F/T sensor response length, got %d want >= %d", len(params), need)
	}
	vals := make([]float64, 0, ftSensorValueCount)
	for i := range ftSensorValueCount {
		idx := i*4 + 1
		vals = append(vals, float64(rutils.Float32FromBytesLE(params[idx:idx+4])))
	}
	return vals, nil
}

func (x *xArm) getFTSensorData(ctx context.Context) ([]float64, error) {
	c := x.newCmd(regMap["FTSensorData"])
	resp, err := x.send(ctx, c, true)
	if err != nil {
		return nil, err
	}
	return parseFTSensorData(resp.params)
}

func (x *xArm) setFTSensorZero(ctx context.Context) error {
	c := x.newCmd(regMap["FTSensorZero"])
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) setCollisionDetectionSensitivity(ctx context.Context, sensitivity int) error {
	c := x.newCmd(regMap["Sensitivity"])
	c.params = append(c.params, byte(sensitivity))
	_, err := x.send(ctx, c, true)
	return err
}
