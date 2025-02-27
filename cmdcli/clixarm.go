package main

import (
	"context"
	"flag"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"

	xarm "github.com/viam-modules/viam-ufactory-xarm/arm"
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

	flag.StringVar(&c.Host, "host", c.Host, "host")
	flag.IntVar(&c.Port, "port", c.Port, "port")
	flag.IntVar(&bj, "bad-joint", bj, "bad joint")
	flag.BoolVar(&debug, "debug", debug, "debug")

	flag.Parse()

	if bj >= 0 {
		c.BadJoints = append(c.BadJoints, bj)
	}

	if debug {
		logger.SetLevel(logging.DEBUG)
	}

	_, err := c.Validate("")
	if err != nil {
		return err
	}

	a, err := xarm.NewXArm(ctx, arm.Named("foo"), &c, logger, xarm.ModelName6DOF)
	if err != nil {
		return err
	}
	defer a.Close(ctx)

	pos, err := a.JointPositions(ctx, nil)
	if err != nil {
		return err
	}

	logger.Infof("positions: %v", pos)

	return nil
}
