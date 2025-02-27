package arm

import (
	"context"
	"testing"

	"github.com/golang/geo/r3"
	pb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	frame "go.viam.com/rdk/referenceframe"
	spatial "go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

var (
	home7 = frame.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})
	wbY   = -426.
)

// This will test solving the path to write the word "VIAM" on a whiteboard.
func TestWriteViam(t *testing.T) {
	fs := frame.NewEmptyFrameSystem("test")

	ctx := context.Background()
	logger := logging.NewTestLogger(t)
	m, err := frame.ParseModelJSONFile("xarm7_kinematics.json", "")
	test.That(t, err, test.ShouldBeNil)

	err = fs.AddFrame(m, fs.World())
	test.That(t, err, test.ShouldBeNil)

	markerOriginFrame, err := frame.NewStaticFrame(
		"marker_origin",
		spatial.NewPoseFromOrientation(&spatial.OrientationVectorDegrees{OY: -1, OZ: 1}),
	)
	test.That(t, err, test.ShouldBeNil)
	markerFrame, err := frame.NewStaticFrame("marker", spatial.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 160}))
	test.That(t, err, test.ShouldBeNil)
	err = fs.AddFrame(markerOriginFrame, m)
	test.That(t, err, test.ShouldBeNil)
	err = fs.AddFrame(markerFrame, markerOriginFrame)
	test.That(t, err, test.ShouldBeNil)

	eraserOriginFrame, err := frame.NewStaticFrame(
		"eraser_origin",
		spatial.NewPoseFromOrientation(&spatial.OrientationVectorDegrees{OY: 1, OZ: 1}),
	)
	test.That(t, err, test.ShouldBeNil)
	eraserFrame, err := frame.NewStaticFrame("eraser", spatial.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: 160}))
	test.That(t, err, test.ShouldBeNil)
	err = fs.AddFrame(eraserOriginFrame, m)
	test.That(t, err, test.ShouldBeNil)
	err = fs.AddFrame(eraserFrame, eraserOriginFrame)
	test.That(t, err, test.ShouldBeNil)

	// draw pos start
	goal := spatial.NewPoseFromProtobuf(&pb.Pose{
		X:  230,
		Y:  wbY + 10,
		Z:  600,
		OY: -1,
	})

	seedMap := map[string][]frame.Input{}
	seedMap[m.Name()] = home7

	plan, err := motionplan.PlanMotion(ctx, &motionplan.PlanRequest{
		Logger: logger,
		Goals: []*motionplan.PlanState{
			motionplan.NewPlanState(frame.FrameSystemPoses{eraserFrame.Name(): frame.NewPoseInFrame(frame.World, goal)}, nil),
		},
		StartState:  motionplan.NewPlanState(nil, seedMap),
		FrameSystem: fs,
	})
	test.That(t, err, test.ShouldBeNil)

	opt := map[string]interface{}{"motion_profile": motionplan.LinearMotionProfile}
	goToGoal := func(seedMap map[string][]frame.Input, goal spatial.Pose) map[string][]frame.Input {
		plan, err := motionplan.PlanMotion(ctx, &motionplan.PlanRequest{
			Logger: logger,
			Goals: []*motionplan.PlanState{
				motionplan.NewPlanState(frame.FrameSystemPoses{eraserFrame.Name(): frame.NewPoseInFrame(frame.World, goal)}, nil),
			},
			StartState:  motionplan.NewPlanState(nil, seedMap),
			FrameSystem: fs,
			Options:     opt,
		})
		test.That(t, err, test.ShouldBeNil)
		return plan.Trajectory()[len(plan.Trajectory())-1]
	}

	seed := plan.Trajectory()[len(plan.Trajectory())-1]
	for _, goal = range viamPoints {
		seed = goToGoal(seed, goal)
	}
}

var viamPoints = []spatial.Pose{
	spatial.NewPoseFromProtobuf(&pb.Pose{X: 200, Y: wbY + 1.5, Z: 595, OY: -1}),
	spatial.NewPoseFromProtobuf(&pb.Pose{X: 120, Y: wbY + 1.5, Z: 595, OY: -1}),
}
