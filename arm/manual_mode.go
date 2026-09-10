package arm

import (
	"context"
	"errors"
	"sync"
	"time"
)

// exitTimer schedules a one-shot automatic exit from manual mode.
type exitTimer struct {
	mu sync.Mutex
	t  *time.Timer
}

func (e *exitTimer) schedule(d time.Duration, exit func()) {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.t != nil {
		e.t.Stop()
	}
	e.t = time.AfterFunc(d, exit)
}

func (e *exitTimer) cancel() {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.t != nil {
		e.t.Stop()
		e.t = nil
	}
}

func (x *xArm) ManualMode(ctx context.Context, extra map[string]any) (bool, error) {
	return x.started.Load() == int32(manualMode), nil
}

func (x *xArm) SetManualMode(ctx context.Context, on bool, enabledFor time.Duration, extra map[string]any) error {
	// Entering freedrive mid-motion would leave the arm compliant while it is still
	// moving; reject it and let the caller stop the motion first.
	if on && x.opMgr.OpRunning() {
		return errors.New("cannot enter manual mode: a motion is in progress")
	}
	x.manualExit.cancel()
	if !on {
		return x.exitManualMode(ctx)
	}
	if err := x.enterManualMode(ctx); err != nil {
		return err
	}
	if enabledFor > 0 {
		x.manualExit.schedule(enabledFor, x.autoExitManualMode)
	}
	return nil
}

// autoExitTimeout bounds the controller I/O for a timer-driven exit from manual
// mode. The exitTimer callback carries no caller context to inherit a deadline
// from, so it needs its own; 10s matches the SDKs' default request timeout.
const autoExitTimeout = 10 * time.Second

// autoExitManualMode runs from the exitTimer when an enabledFor deadline expires.
func (x *xArm) autoExitManualMode() {
	if x.closed.Load() || x.started.Load() != int32(manualMode) {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), autoExitTimeout)
	defer cancel()
	if err := x.exitManualMode(ctx); err != nil {
		x.logger.Errorf("failed to automatically exit manual mode: %v", err)
	}
}
