// Package main implements an xarm module
package main

import (
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	xarm "viam-xarm/arm"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: xarm.XArm6Model},
		resource.APIModel{API: arm.API, Model: xarm.XArm7Model},
		resource.APIModel{API: arm.API, Model: xarm.XArmLite6Model},
	)
}
