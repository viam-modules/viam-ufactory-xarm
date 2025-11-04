// Package main implements an xarm module
package main

import (
	xarm "github.com/viam-modules/viam-ufactory-xarm/arm"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: arm.API, Model: xarm.XArm6Model},
		resource.APIModel{API: arm.API, Model: xarm.XArm7Model},
		resource.APIModel{API: arm.API, Model: xarm.XArmLite6Model},
		resource.APIModel{API: gripper.API, Model: xarm.GripperModel},
		resource.APIModel{API: gripper.API, Model: xarm.GripperModelLite},
		resource.APIModel{API: gripper.API, Model: xarm.VacuumGripperModel},
	)
}
