package arm

import (
	"context"
	"fmt"
)

// Controller C23, distinct from servo S23 (motor position deviation).
const errCodeJointLimit = 0x17

func (x *xArm) clearJointLimitError(ctx context.Context) (bool, error) {
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if x.opMgr.OpRunning() || x.started.Load() == int32(manualMode) {
		return false, fmt.Errorf("joint-limit clearing requires an idle arm outside manual mode")
	}
	state, err := x.getErrorParams(ctx)
	if err != nil {
		return false, err
	}
	if state[0]&(errorState|warningState) == 0 && state[1] == 0 && state[2] == 0 {
		return false, nil
	}
	if state[1] != errCodeJointLimit || state[2] != 0 || state[0]&(errorState|warningState) != errorState {
		return false, decodeError(state)
	}
	status, err := x.send(ctx, x.newCmd(regMap["GetState"]), false)
	if err != nil {
		return false, err
	}
	// Viam stops failed execution into state 3. Clearing its latched C23 does
	// not enable motion; the caller still owns cancellation and replanning.
	if len(status.params) < 2 || (status.params[1] != 2 && status.params[1] != 3 && status.params[1] != 4) {
		return false, fmt.Errorf("controller is not stationary for joint-limit clearing: %v", status.params)
	}
	state, err = x.getErrorParams(ctx)
	if err != nil {
		return false, err
	}
	if state[1] != errCodeJointLimit || state[2] != 0 || state[0]&(errorState|warningState) != errorState {
		return false, fmt.Errorf("controller fault changed before clearing: %v", state)
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	// Do not use checkReadyState: it also clears unrelated faults and warnings.
	if _, err := x.send(ctx, x.newCmd(regMap["ClearError"]), false); err != nil {
		return false, err
	}
	x.started.Store(-1)
	state, err = x.getErrorParams(ctx)
	if err != nil {
		return false, err
	}
	if state[0]&(errorState|warningState) != 0 || state[1] != 0 || state[2] != 0 {
		return false, decodeError(state)
	}
	return true, nil
}
