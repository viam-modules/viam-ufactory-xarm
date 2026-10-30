// Command setjoints sets the scenecheck replica's fake arm joints, in degrees.
package main

import (
	"context"
	"os"
	"strconv"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/utils"
	rpc "go.viam.com/utils/rpc"
)

func main() {
	ctx := context.Background()
	r, err := client.New(ctx, "localhost:18080", logging.NewLogger("m"), client.WithDialOptions(rpc.WithInsecure(), rpc.WithDisableDirectGRPC()))
	if err != nil {
		panic(err)
	}
	defer func() { _ = r.Close(ctx) }() //nolint:errcheck
	a, err := arm.FromProvider(r, "arm")
	if err != nil {
		panic(err)
	}
	var in []referenceframe.Input
	for _, s := range os.Args[1:] {
		v, err := strconv.ParseFloat(s, 64)
		if err != nil {
			panic(err)
		}
		in = append(in, utils.DegToRad(v))
	}
	if err := a.MoveToJointPositions(ctx, in, nil); err != nil {
		panic(err)
	}
}
