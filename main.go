// Package main implements an xarm module
package main

import (
	xarm "viam-xarm/arm"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

const moduleName = "UFactory xArm Go Module"

func main() {
	module.ModularMain(
		moduleName,
		resource.APIModel{API: arm.API, Model: xarm.XArm6Model},
		resource.APIModel{API: arm.API, Model: xarm.XArm7Model},
		resource.APIModel{API: arm.API, Model: xarm.XArmLite6Model},
	)
}
