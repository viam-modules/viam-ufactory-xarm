package arm

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"time"

	"go.uber.org/multierr"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	rutils "go.viam.com/rdk/utils"
	"go.viam.com/utils"
)

const errCodeCollision = 0x1F
const servoMotionMode = 1

var servoErrorMap = map[byte]string{
	0x00: "xArm Servo: Joint Communication Error",
	0x0A: "xArm Servo: Current Detection Error",
	0x0B: "xArm Servo: Joint Overcurrent",
	0x0C: "xArm Servo: Joint Overspeed",
	0x0E: "xArm Servo: Position Command Overlimit",
	0x0F: "xArm Servo: Joints Overheat",
	0x10: "xArm Servo: Encoder Initialization Error",
	0x11: "xArm Servo: Single-turn Encoder Error",
	0x12: "xArm Servo: Multi-turn Encoder Error",
	0x13: "xArm Servo: Low Battery Voltage",
	0x14: "xArm Servo: Driver IC Hardware Error",
	0x15: "xArm Servo: Driver IC Init Error",
	0x16: "xArm Servo: Encoder Config Error",
	0x17: "xArm Servo: Large Motor Position Deviation",
	0x1A: "xArm Servo: Joint N Positive Overrun",
	0x1B: "xArm Servo: Joint N Negative Overrun",
	0x1C: "xArm Servo: Joint Commands Error",
	0x21: "xArm Servo: Drive Overloaded",
	0x22: "xArm Servo: Motor Overload",
	0x23: "xArm Servo: Motor Type Error",
	0x24: "xArm Servo: Driver Type Error",
	0x27: "xArm Servo: Joint Overvoltage",
	0x28: "xArm Servo: Joint Undervoltage",
	0x31: "xArm Servo: EEPROM RW Error",
	0x34: "xArm Servo: Initialization of Motor Angle Error",
}

var armBoxErrorMap = map[byte]string{
	0x01:             "xArm: Emergency Stop Button Pushed In",
	0x02:             "xArm: Emergency IO Triggered",
	0x03:             "xArm: Emergency Stop 3-State Switch Pressed",
	0x0B:             "xArm: Power Cycle Required",
	0x0C:             "xArm: Power Cycle Required",
	0x0D:             "xArm: Power Cycle Required",
	0x0E:             "xArm: Power Cycle Required",
	0x0F:             "xArm: Power Cycle Required",
	0x10:             "xArm: Power Cycle Required",
	0x11:             "xArm: Power Cycle Required",
	0x13:             "xArm: Gripper Communication Error",
	0x15:             "xArm: Kinematic Error",
	0x16:             "xArm: Self Collision Error",
	0x17:             "xArm: Joint Angle Exceeds Limit",
	0x18:             "xArm: Speed Exceeds Limit",
	0x19:             "xArm: Planning Error",
	0x1A:             "xArm: Linux RT Error",
	0x1B:             "xArm: Command Reply Error",
	0x1C:             "xArm: End Module Communication Error",
	0x1D:             "xArm: Other Errors",
	0x1E:             "xArm: Feedback Speed Exceeds Limit",
	errCodeCollision: "xArm: Collision Caused Abnormal Current",
	0x20:             "xArm: Three-point Drawing Circle Calculation Error",
	0x21:             "xArm: Abnormal Arm Current",
	0x22:             "xArm: Recording Timeout",
	0x23:             "xArm: Safety Boundary Limit",
	0x24:             "xArm: Delay Command Limit Exceeded",
	0x25:             "xArm: Abnormal Motion in Manual Mode",
	0x26:             "xArm: Abnormal Joint Angle",
	0x27:             "xArm: Abnormal Communication Between Power Boards",
}

var armBoxWarnMap = map[byte]string{
	0x0B: "xArm Warning: Buffer Overflow",
	0x0C: "xArm Warning: Command Parameter Abnormal",
	0x0D: "xArm Warning: Unknown Command",
	0x0E: "xArm Warning: Command No Solution",
}

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
	"ToggleBrake":    0x12,
	"SetMode":        0x13,
	"MoveJoints":     0x1D,
	"ZeroJoints":     0x19,
	"JointPos":       0x2A,
	"Sensitivity":    0x25,
	"SetBound":       0x34,
	"EnableBound":    0x34,
	"CurrentTorque":  0x37,
	"SetEEModel":     0x4E,
	"ServoError":     0x6A,
	"GripperControl": 0x7C,
	"VacuumControl":  0x7F,
	"LoadID":         0xCC,
	"VacuumState":    0x80,
}

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

func (x *xArm) newCmd(reg byte) cmd {
	x.tid++
	return cmd{tid: x.tid, prot: 2, reg: reg}
}

func (x *xArm) send(ctx context.Context, c cmd, checkError bool) (cmd, error) {
	if x.closed.Load() {
		return cmd{}, errors.New("closed")
	}

	resp, err := x.writeBytes(ctx, c)
	if err != nil {
		return cmd{}, err
	}

	// check the error returned by the response
	if checkError {
		state := resp.params[0]
		// the 2nd and 3rd MSB in state byte indicate if there
		// is an error or warning respectively.
		if state&(1<<6|1<<5) != 0 {
			params, err := x.getErrorParams(ctx)
			if err != nil {
				return cmd{}, err
			}
			errCode := params[1]
			if errCode == errCodeCollision {
				// overcurrent estop has occurred, must be manually cleared by user.
				return cmd{}, fmt.Errorf("collision caused overcurrent: ensure robot is clear of obstacles and clear error " +
					"through UFACTORY Studio or clear_error do command")
			}
			// Any other errors are cleared automatically by the driver.
			return cmd{}, multierr.Combine(
				decodeError(params),
				x.resetErrorState(ctx))
		}
	}
	return resp, err
}

func (x *xArm) writeBytes(ctx context.Context, c cmd) (cmd, error) {
	x.moveLock.Lock()
	defer x.moveLock.Unlock()

	if x.conn == nil { // THIS HAS TO BE DONE INSIDE THE LOCK
		err := x.connect(ctx)
		if err != nil {
			x.resetConnection()
			return cmd{}, err
		}
	}

	b := c.bytes()
	// add deadline so we aren't waiting forever
	if err := x.conn.SetDeadline(time.Now().Add(5 * time.Second)); err != nil {
		x.resetConnection()
		return cmd{}, err
	}
	_, err := x.conn.Write(b)
	if err != nil {
		x.resetConnection()
		return cmd{}, err
	}
	return x.responseInLock(ctx)
}

func (x *xArm) responseInLock(ctx context.Context) (cmd, error) {
	buf, err := utils.ReadBytes(ctx, x.conn, 7)
	if err != nil {
		x.resetConnection()
		return cmd{}, err
	}
	c := cmd{}
	c.tid = binary.BigEndian.Uint16(buf[0:2])
	c.prot = binary.BigEndian.Uint16(buf[2:4])
	c.reg = buf[6]
	length := binary.BigEndian.Uint16(buf[4:6])
	c.params, err = utils.ReadBytes(ctx, x.conn, int(length-1))
	if err != nil {
		x.resetConnection()
		return cmd{}, err
	}
	return c, err
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

func (x *xArm) resetErrorState(ctx context.Context) error {
	c1 := x.newCmd(regMap["ClearError"])
	c2 := x.newCmd(regMap["ClearWarn"])
	_, err1 := x.send(ctx, c1, false)
	_, err2 := x.send(ctx, c2, false)
	err3 := x.setMotionMode(ctx, servoMotionMode)
	err4 := x.setMotionState(ctx, 0)
	return multierr.Combine(err1, err2, err3, err4)
}

func (x *xArm) getErrorParams(ctx context.Context) ([]byte, error) {
	c := x.newCmd(regMap["GetError"])
	resp, err := x.writeBytes(ctx, c)
	if err != nil {
		return nil, err
	}

	if len(resp.params) < 3 {
		return nil, errors.New("bad arm error query response")
	}

	return resp.params, nil
}

func decodeError(params []byte) error {
	errCode := params[1]
	warnCode := params[2]
	errMsg, isErr := armBoxErrorMap[errCode]
	warnMsg, isWarn := armBoxWarnMap[warnCode]
	if isErr || isWarn {
		return multierr.Combine(errors.New(errMsg),
			errors.New(warnMsg))
	}

	// Commands are returning error codes that are not mentioned in the
	// developer manual
	return errors.New("xArm: UNKNOWN ERROR")
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

// toggleBrake toggles the brakes on or off.
// True disengages brakes, false engages them.
func (x *xArm) toggleBrake(ctx context.Context, disable bool) error {
	c := x.newCmd(regMap["ToggleBrake"])
	var enByte byte
	if disable {
		enByte = 1
	}
	c.params = append(c.params, 8, enByte)
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) start(ctx context.Context) error {
	if x.started.Load() {
		return nil
	}

	err := x.toggleServos(ctx, true)
	if err != nil {
		return err
	}
	err = x.setMotionMode(ctx, servoMotionMode)
	if err != nil {
		return err
	}
	if err := x.setMotionState(ctx, 0); err != nil {
		return err
	}
	x.started.Store(true)
	return nil
}

// motionStopped will check if all arm pieces have stopped moving.
func (x *xArm) motionStopped(ctx context.Context) (bool, error) {
	c := x.newCmd(regMap["GetState"])
	sData, err := x.send(ctx, c, true)
	if err != nil {
		return false, err
	}
	if len(sData.params) < 2 {
		return false, errors.New("malformed state data response in motionStopped")
	}
	if sData.params[1] != 1 {
		return true, nil
	}
	return false, nil
}

// Close shuts down the arm servos and engages brakes.
func (x *xArm) Close(ctx context.Context) error {
	x.closed.Store(true)
	if x.conn == nil {
		return nil
	}

	if err := x.toggleBrake(ctx, false); err != nil {
		return err
	}
	if err := x.toggleServos(ctx, false); err != nil {
		return err
	}
	if err := x.setMotionState(ctx, 4); err != nil {
		return err
	}

	err := x.conn.Close()
	if err != nil {
		return err
	}
	x.conn = nil

	return nil
}

// MoveToJointPositions moves the arm to the requested joint positions.
func (x *xArm) MoveToJointPositions(ctx context.Context, newPositions []referenceframe.Input, extra map[string]interface{}) error {
	ctx, done := x.opMgr.New(ctx)
	defer done()
	return x.GoToInputs(ctx, newPositions)
}

func (x *xArm) MoveThroughJointPositions(
	ctx context.Context,
	positions [][]referenceframe.Input,
	opts *arm.MoveOptions,
	_ map[string]interface{},
) error {
	if opts != nil {
		// Ensure move options are within the valid range
		opts.MaxVelRads = x.clampMoveOptions(
			opts.MaxVelRads,
			rutils.DegToRad(minSpeed),
			rutils.DegToRad(maxSpeed),
			"max velocity",
		)

		opts.MaxAccRads = x.clampMoveOptions(
			opts.MaxAccRads,
			0,
			rutils.DegToRad(maxAccel),
			"max acceleration",
		)
	}
	// set to servo motion mode
	if err := x.setMotionMode(ctx, servoMotionMode); err != nil {
		return err
	}

	// Ensure the robot is in correct state to move
	b, err := x.getErrorParams(ctx)
	if err != nil {
		return err
	}
	state := b[0]

	// xarm is not in movement state: was just restarted or estopped
	// must set state back to motion
	if state == 0x10 {
		if err = x.setMotionState(ctx, 0); err != nil {
			return err
		}
	}
	for _, goal := range positions {
		// check that joint positions are not out of bounds
		if err := arm.CheckDesiredJointPositions(ctx, x, goal); err != nil {
			return err
		}
	}
	curPos, err := x.JointPositions(ctx, nil)
	if err != nil {
		return err
	}
	armRawSteps, err := x.createRawJointSteps(curPos, positions, opts)
	if err != nil {
		return err
	}
	return x.executeInputs(ctx, armRawSteps)
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
	opts *arm.MoveOptions,
) (
	[][]float64, error) {
	x.confLock.Lock()
	speed := x.speed
	acceleration := x.acceleration
	x.confLock.Unlock()

	// If move options were given, use those instead.
	if opts != nil {
		if opts.MaxVelRads != 0 {
			speed = opts.MaxVelRads
		}
		if opts.MaxAccRads != 0 {
			acceleration = opts.MaxAccRads
		}
	}

	// Generate list of joint positions to pass through
	// This is almost-calculus but not quite because it's explicitly discretized
	accelStep := acceleration / x.moveHZ
	interwaypointAccelStep := interwaypointAccel / x.moveHZ

	from := referenceframe.InputsToFloats(startInputs)

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
		to := referenceframe.InputsToFloats(toInputs)
		maxVal := floatMaxDiff(from, to)
		displacementTotal += maxVal
		nSteps := (math.Abs(maxVal) / speed) * x.moveHZ
		stepTotal += nSteps
		from = to
	}

	nominalAccelSteps := int((speed / acceleration) * x.moveHZ) // This many steps to accelerate, and the same to decelerate
	if float64(nominalAccelSteps) > stepTotal*0.95 {
		nominalAccelSteps = int(0.95 * math.Sqrt(displacementTotal/acceleration) * x.moveHZ)
	}
	maxVel := (float64(nominalAccelSteps) / x.moveHZ) * acceleration

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
		currSpeed := accelStep
		steps := [][]float64{}
		from = referenceframe.InputsToFloats(startInputs)
		lastInputs := startInputs
		for i, toInputs := range allInputSteps {
			to := referenceframe.InputsToFloats(toInputs)
			runningFrom := from

			for currDiff := floatMaxDiff(runningFrom, to); currDiff > 1e-6; currDiff = floatMaxDiff(runningFrom, to) {
				if currSpeed <= 0 {
					break
				}
				nSteps := (currDiff / currSpeed) * x.moveHZ
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
				runningFrom = referenceframe.InputsToFloats(nextInputs)
				steps = append(steps, referenceframe.InputsToFloats(nextInputs))

				if currSpeed < speed {
					currSpeed += accelStep * stepSize
					if currSpeed > speed {
						currSpeed = speed
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
			from = to
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
			accelInputSteps = append(accelInputSteps, referenceframe.FloatsToInputs(decelSteps[len(decelSteps)-1]))
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

func (x *xArm) executeInputs(ctx context.Context, rawSteps [][]float64) error {
	if err := x.start(ctx); err != nil {
		return err
	}
	// convenience for structuring and sending individual joint steps
	for _, step := range rawSteps {
		c := x.newCmd(regMap["MoveJoints"])
		jFloatBytes := make([]byte, 4)
		for _, jRad := range step {
			binary.LittleEndian.PutUint32(jFloatBytes, math.Float32bits(float32(jRad)))
			c.params = append(c.params, jFloatBytes...)
		}
		// xarm 6 has 6 joints, but protocol needs 7- add 4 bytes for a blank 7th joint
		for dof := x.dof; dof < 7; dof++ {
			c.params = append(c.params, 0, 0, 0, 0)
		}
		// When in servoj mode, motion time, speed, and acceleration are not handled by the control box
		c.params = append(c.params, 0, 0, 0, 0)
		c.params = append(c.params, 0, 0, 0, 0)
		c.params = append(c.params, 0, 0, 0, 0)
		_, err := x.send(ctx, c, true)
		if err != nil {
			return err
		}
		if !utils.SelectContextOrWait(ctx, time.Duration(1000000./x.moveHZ)*time.Microsecond) {
			return ctx.Err()
		}
	}

	return nil
}

// EndPosition computes and returns the current cartesian position.
func (x *xArm) EndPosition(ctx context.Context, extra map[string]interface{}) (spatialmath.Pose, error) {
	joints, err := x.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return referenceframe.ComputeOOBPosition(x.model, joints)
}

// MoveToPosition moves the arm to the specified cartesian position.
func (x *xArm) MoveToPosition(ctx context.Context, pos spatialmath.Pose, extra map[string]interface{}) error {
	ctx, done := x.opMgr.New(ctx)
	defer done()
	if err := x.start(ctx); err != nil {
		return err
	}
	if err := motion.MoveArm(ctx, x.logger, x, pos); err != nil {
		return err
	}
	return x.opMgr.WaitForSuccess(
		ctx,
		time.Millisecond*50,
		x.motionStopped,
	)
}

// JointPositions returns the current positions of all joints.
func (x *xArm) JointPositions(ctx context.Context, extra map[string]interface{}) ([]referenceframe.Input, error) {
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
	return referenceframe.FloatsToInputs(radians), nil
}

// Stop stops the xArm but also reinitializes the arm so it can take commands again.
func (x *xArm) Stop(ctx context.Context, extra map[string]interface{}) error {
	ctx, done := x.opMgr.New(ctx)
	defer done()

	x.started.Store(false)

	if err := x.setMotionState(ctx, 3); err != nil {
		return err
	}

	return x.start(ctx)
}

// IsMoving returns whether the arm is moving.
func (x *xArm) IsMoving(ctx context.Context) (bool, error) {
	return x.opMgr.OpRunning(), nil
}

func (x *xArm) setupGripper(ctx context.Context) error {
	if err := x.enableGripper(ctx); err != nil {
		return err
	}
	if err := x.setGripperMode(ctx, false); err != nil {
		return err
	}
	return nil
}

func (x *xArm) gripperPreamble(write bool) cmd {
	c := x.newCmd(regMap["GripperControl"])
	c.params = append(c.params, 0x09) // host id
	c.params = append(c.params, 0x08) // gripper id
	if write {
		c.params = append(c.params, 0x10)
	} else {
		c.params = append(c.params, 0x03)
	}
	return c
}

func (x *xArm) enableGripper(ctx context.Context) error {
	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x01, 0x00)
	c.params = append(c.params, 0x00, 0x01)
	c.params = append(c.params, 0x02)
	c.params = append(c.params, 0x00, 0x01)
	_, err := x.send(ctx, c, true)
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
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) setGripperPosition(ctx context.Context, position uint32) error {
	c := x.gripperPreamble(true)
	c.params = append(c.params, 0x07, 0x00)
	c.params = append(c.params, 0x00, 0x02)
	c.params = append(c.params, 0x04)
	tmpBytes := make([]byte, 4)
	binary.BigEndian.PutUint32(tmpBytes, position)
	x.logger.Debugf("setGripperPosition bytes", tmpBytes)
	c.params = append(c.params, tmpBytes...)
	_, err := x.send(ctx, c, true)
	return err
}

func (x *xArm) getGripperPosition(ctx context.Context) (uint32, error) {
	c := x.gripperPreamble(false)
	c.params = append(c.params, 0x07, 0x02)
	c.params = append(c.params, 0x00, 0x02)
	res, err := x.send(ctx, c, true)
	if err != nil {
		return 0, err
	}

	x.logger.Debugf("getGripperPosition: %v %v", res, res.params)

	// open  : 0 9 8 3 4 0 0 3 73
	// closed: 0 9 8 3 4 0 0 0 0
	if len(res.params) != 9 {
		return 0, fmt.Errorf("weird length for getGripperPosition response: %d %v", len(res.params), res.params)
	}
	return binary.BigEndian.Uint32(res.params[5:]), nil
}

func (x *xArm) getVacuumStatus(ctx context.Context) (bool, error) {
	c := x.newCmd(regMap["VacuumState"])
	additionalParams := []byte{
		0x09,
		0x0A,
		0x14,
	}
	c.params = append(c.params, additionalParams...)
	res, err := x.send(ctx, c, true)
	if err != nil {
		return false, err
	}

	if len(res.params) != 5 {
		return false, fmt.Errorf("weird length for getVacuumStatus response: %d %v", len(res.params), res.params)
	}
	return (res.params[4] == 0x01), nil
}

// This is the host ID and gripper address which should be appended to each command.
func (x *xArm) vacuumPreamble() cmd {
	c := x.newCmd(regMap["VacuumControl"])

	c.params = append(c.params, 0x09) // host ID
	c.params = append(c.params, 0x0A) // vacuum ID
	c.params = append(c.params, 0x15)
	return c
}

// Grab maps to open in ufactory.
func (x *xArm) grabVacuum(ctx context.Context) error {
	// Ufactory requires opening channel 0 and channel 1
	// to open the vacuum gripper
	c1 := x.vacuumPreamble()
	c1.params = append(c1.params,
		0x00,
		0x80,
		0x80,
		0x43,
	)
	_, err := x.send(ctx, c1, true)
	if err != nil {
		return err
	}

	c2 := x.vacuumPreamble()
	c2.params = append(c2.params,
		0x00,
		0x00,
		0x00,
		0x44,
	)
	_, err = x.send(ctx, c2, true)
	if err != nil {
		return err
	}
	return nil
}

// Close maps to open in ufactory.
func (x *xArm) openVacuum(ctx context.Context) error {
	// Ufactory requires close channel 0 and channel 1
	// to stop the vacuum gripper
	c1 := x.vacuumPreamble()
	c1.params = append(c1.params,
		0x00,
		0x00,
		0x80,
		0x43,
	)
	_, err := x.send(ctx, c1, true)
	if err != nil {
		return err
	}

	c2 := x.vacuumPreamble()
	c2.params = append(c2.params,
		0x00,
		0x80,
		0x00,
		0x44,
	)
	_, err = x.send(ctx, c2, true)
	if err != nil {
		return err
	}
	return nil
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

func (x *xArm) setCollisionDetectionSensitivity(ctx context.Context, sensitivity int) error {
	c := x.newCmd(regMap["Sensitivity"])
	c.params = append(c.params, byte(sensitivity))
	_, err := x.send(ctx, c, true)
	return err
}
