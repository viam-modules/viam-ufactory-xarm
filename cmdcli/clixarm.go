// Package main for testing xarm stuff
package main

import (
	"context"
	"flag"

	xarm "github.com/viam-modules/viam-ufactory-xarm/arm"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	rutils "go.viam.com/rdk/utils"
	"go.viam.com/utils"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewLogger("xarmcli")

	c := xarm.Config{
		Host:         "",
		Port:         0,
		BadJoints:    []int{},
		Speed:        0,
		Acceleration: 0,
	}

	bj := -1
	debug := false
	moveJoint := -1
	moveAmount := 5.0
	extra := map[string]interface{}{}

	flag.StringVar(&c.Host, "host", c.Host, "host")
	flag.IntVar(&c.Port, "port", c.Port, "port")
	flag.IntVar(&bj, "bad-joint", bj, "bad joint")
	flag.IntVar(&moveJoint, "move-joint", moveJoint, "joint to move")
	flag.Float64Var(&moveAmount, "move-amount", moveAmount, "amount to move in degrees")
	flag.BoolVar(&debug, "debug", debug, "debug")
	flag.Float64Var(&c.Speed, "speed", c.Speed, "speed in degs per second")
	flag.Float64Var(&c.Acceleration, "acceleration", c.Acceleration, "acceleration in degs per sec^2")

	direct := flag.Bool("direct", false, "")
	doNotWait := flag.Bool("do-not-wait", false, "")

	flag.Parse()

	if bj >= 0 {
		c.BadJoints = append(c.BadJoints, bj)
	}

	if debug {
		logger.SetLevel(logging.DEBUG)
	}

	if *direct {
		extra["direct"] = true
	}
	if *doNotWait {
		extra["waitAtEnd"] = false
	}

	_, _, err := c.Validate("")
	if err != nil {
		return err
	}

	a, err := xarm.NewXArm(ctx, arm.Named("foo"), &c, logger, xarm.ModelName6DOF, nil)
	if err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(func() error {
		return a.Close(ctx)
	})

	pos, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}

	logger.Infof("positions: %v", pos)

	if moveJoint >= 0 {
		pos[moveJoint] += rutils.DegToRad(moveAmount)
		logger.Infof("moving to: %v", pos)
		err = a.MoveToJointPositions(ctx, pos, extra)
		if err != nil {
			return err
		}
	}

	return nil
}
