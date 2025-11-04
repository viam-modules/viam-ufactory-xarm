package mptests

import (
	"testing"

	"github.com/golang/geo/r3"
	xarm "github.com/viam-modules/viam-ufactory-xarm/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func makeTestFrameSystem(logger logging.Logger) (*referenceframe.FrameSystem, error) {
	armModel, err := xarm.MakeModelFrame(xarm.ModelName6DOF, nil, nil, logger)
	if err != nil {
		return nil, err
	}

	parts := []*referenceframe.FrameSystemPart{

		{ // arm
			FrameConfig: referenceframe.NewLinkInFrame(
				"world",
				spatialmath.NewPose(
					r3.Vector{Y: 150},
					&spatialmath.OrientationVectorDegrees{Theta: 30},
				),
				"arm-right",
				nil,
			),
			ModelFrame: armModel,
		},

		{ // gripper
			FrameConfig: referenceframe.NewLinkInFrame(
				"arm-right",
				spatialmath.NewPose(
					r3.Vector{Z: 150},
					&spatialmath.OrientationVectorDegrees{Theta: 30},
				),
				"gripper-right",
				nil,
			),

			ModelFrame: referenceframe.NewSimpleModel("foo"),
		},
	}

	return referenceframe.NewFrameSystem("temp", parts, nil)
}

func BenchmarkMP1(b *testing.B) {
	ctx := b.Context()
	logger := logging.NewTestLogger(b)

	fs, err := makeTestFrameSystem(logger)
	test.That(b, err, test.ShouldBeNil)

	logger.Infof("fs: %v", fs)

	startJoints := []referenceframe.Input{
		{-1.6046726703643799},
		{-0.9392223954200745},
		{-0.28884029388427734},
		{4.769320487976074},
		{1.0797568559646606},
		{-2.8038926124572754},
	}

	dest := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(r3.Vector{X: 191.391061, Y: 297.871836, Z: 371.730225},
		&spatialmath.OrientationVectorDegrees{OX: 0.801501, OY: -0.597993, OZ: -0.000224, Theta: 101.891328}))

	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		planReq := &armplanning.PlanRequest{
			FrameSystem: fs,
			Goals: []*armplanning.PlanState{
				armplanning.NewPlanState(referenceframe.FrameSystemPoses{"gripper-right": dest}, nil),
			},
			StartState: armplanning.NewPlanState(nil, referenceframe.FrameSystemInputs{"arm-right": startJoints}),
		}

		plan, _, err := armplanning.PlanMotion(ctx, logger, planReq)
		test.That(b, err, test.ShouldBeNil)

		logger.Infof("plan: %v", plan)
	}
}
