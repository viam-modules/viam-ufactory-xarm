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
		Host:      "",
		Port:      0,
		BadJoints: []int{},
	}

	bj := -1
	debug := false
	moveJoint := -1
	moveAmount := 5.0

	flag.StringVar(&c.Host, "host", c.Host, "host")
	flag.IntVar(&c.Port, "port", c.Port, "port")
	flag.IntVar(&bj, "bad-joint", bj, "bad joint")
	flag.IntVar(&moveJoint, "move-joint", moveJoint, "joint to move")
	flag.Float64Var(&moveAmount, "move-amount", moveAmount, "amount to move in degrees")
	flag.BoolVar(&debug, "debug", debug, "debug")

	flag.Parse()

	if bj >= 0 {
		c.BadJoints = append(c.BadJoints, bj)
	}

	if debug {
		logger.SetLevel(logging.DEBUG)
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
		pos[moveJoint].Value += rutils.DegToRad(moveAmount)
		logger.Infof("moving to: %v", pos)
		err = a.MoveToJointPositions(ctx, pos, nil)
		if err != nil {
			return err
		}
	}

	return nil
}
