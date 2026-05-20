package arm

import (
	"errors"
	"fmt"

	"go.uber.org/multierr"
)

const errCodeCollision = 0x1F

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

// armBoxErrorMap maps controller error codes (decimal in UFactory docs, hex
// on the wire) to human-readable descriptions. Source:
// https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api_code.md
var armBoxErrorMap = map[byte]string{
	0x01:             "xArm: Emergency Stop Button Pushed In",
	0x02:             "xArm: Emergency IO Triggered",
	0x03:             "xArm: Emergency Stop 3-State Switch Pressed",
	0x0A:             "xArm: Servo motor error",
	0x0B:             "xArm: Servo motor 1 error",
	0x0C:             "xArm: Servo motor 2 error",
	0x0D:             "xArm: Servo motor 3 error",
	0x0E:             "xArm: Servo motor 4 error",
	0x0F:             "xArm: Servo motor 5 error",
	0x10:             "xArm: Servo motor 6 error",
	0x11:             "xArm: Servo motor 7 error",
	0x12:             "xArm: Force Torque Sensor Communication Error",
	0x13:             "xArm: End Module Communication Error",
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
	0x21:             "xArm: Controller GPIO error",
	0x22:             "xArm: Recording Timeout",
	0x23:             "xArm: Safety Boundary Limit",
	0x24:             "xArm: Delay Command Limit Exceeded",
	0x25:             "xArm: Abnormal Motion in Manual Mode",
	0x26:             "xArm: Abnormal Joint Angle",
	0x27:             "xArm: Abnormal Communication Between Power Boards",
	0x28:             "xArm: No IK available",
	0x32:             "xArm: Six-axis Force Torque Sensor read error",
	0x33:             "xArm: Six-axis Force Torque Sensor set mode error",
	0x34:             "xArm: Six-axis Force Torque Sensor set zero error",
	0x35:             "xArm: Six-axis Force Torque Sensor overloaded / reading exceeds limit",
	0x3C:             "xArm: Linear speed exceeds limit in ServoJ mode",
	0x6E:             "xArm: Robot Arm Base Board Communication Error",
	0x6F:             "xArm: Control Box External 485 Device Communication Error",
}

// armBoxWarnMap maps controller warning codes (decimal in UFactory docs, hex
// on the wire) to human-readable descriptions. Source:
// https://github.com/xArm-Developer/xArm-Python-SDK/blob/master/doc/api/xarm_api_code.md
var armBoxWarnMap = map[byte]string{
	0x0B: "xArm Warning: Buffer Overflow",
	0x0C: "xArm Warning: Command Parameter Abnormal",
	0x0D: "xArm Warning: Unknown Command",
	0x0E: "xArm Warning: Command No Solution",
	0x0F: "xArm Warning: Modbus cmd full",
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
	// developer manual — surface the raw bytes so users can cross-reference
	// them in UFactory's docs / share with support.
	return fmt.Errorf("xArm: UNKNOWN ERROR (state=0x%02x errCode=0x%02x warnCode=0x%02x)",
		params[0], params[1], params[2])
}
