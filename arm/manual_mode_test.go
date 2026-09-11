package arm

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/operation"
	"go.viam.com/test"
)

func TestExitTimerFires(t *testing.T) {
	var et exitTimer
	fired := make(chan struct{})
	et.schedule(10*time.Millisecond, func() { close(fired) })

	select {
	case <-fired:
	case <-time.After(time.Second):
		t.Fatal("timer did not fire within 1s")
	}
}

func TestExitTimerCancel(t *testing.T) {
	var et exitTimer
	fired := make(chan struct{})
	et.schedule(20*time.Millisecond, func() { close(fired) })
	et.cancel()

	select {
	case <-fired:
		t.Fatal("timer fired after cancel")
	case <-time.After(100 * time.Millisecond):
	}
}

func TestExitTimerReschedule(t *testing.T) {
	var et exitTimer
	firstFired := make(chan struct{})
	secondFired := make(chan struct{}, 2)
	et.schedule(50*time.Millisecond, func() { close(firstFired) })
	et.schedule(10*time.Millisecond, func() { secondFired <- struct{}{} })

	select {
	case <-secondFired:
	case <-time.After(time.Second):
		t.Fatal("rescheduled timer did not fire within 1s")
	}

	// Wait past the first timer's original deadline to confirm it was stopped
	// and the second callback ran exactly once.
	time.Sleep(100 * time.Millisecond)
	select {
	case <-firstFired:
		t.Fatal("first timer fired despite being rescheduled")
	default:
	}
	test.That(t, len(secondFired), test.ShouldEqual, 0)
}

func TestSetManualModeRejectsWhileMoving(t *testing.T) {
	x := &xArm{opMgr: operation.NewSingleOperationManager()}
	_, done := x.opMgr.New(context.Background())
	defer done()

	err := x.SetManualMode(context.Background(), true, 0, nil)

	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "motion is in progress")
}
