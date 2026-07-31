package arm

import (
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
)

// kinematicsArtifact identifies the kinematics files to use for an
// (arm model, hardware variant) combination.
type kinematicsArtifact struct {
	json         []byte
	urdfBasename string
	// numMeshes is the count of <mesh> refs in the URDF; 0 for grippers
	// (which use a scalar ratio). Kept in sync with the URDF by hand.
	numMeshes int
	// variant is a short label used only in logs.
	variant string
}

type armVariantKey struct {
	modelName   string
	armTypeCode int
}

var armKinematicsBase = map[string]kinematicsArtifact{
	ModelName6DOF: {json: xArm6modeljson, urdfBasename: "xarm6", numMeshes: 6},
	ModelName7DOF: {json: xArm7modeljson, urdfBasename: "xarm7", numMeshes: 7},
	ModelNameLite: {json: lite6modeljson, urdfBasename: "lite6", numMeshes: 7},
	ModelName850:  {json: xArm850modeljson, urdfBasename: "uf850", numMeshes: 7},
}

// armTypeCode1305 identifies the xArm6 1305 wrist variant (SN "XI1305…").
const armTypeCode1305 = 1305

// armKinematicsVariants overrides the base entry when the detected
// armTypeCode matches. xarm6 and xarm6_1305 share the JSON kinematics;
// only collision meshes differ.
var armKinematicsVariants = map[armVariantKey]kinematicsArtifact{
	{ModelName6DOF, armTypeCode1305}: {
		json: xArm6modeljson, urdfBasename: "xarm6_1305", numMeshes: 7, variant: "1305",
	},
}

func resolveArmKinematicsArtifact(modelName string, detected detectedArm) (kinematicsArtifact, error) {
	if v, ok := armKinematicsVariants[armVariantKey{modelName, detected.armTypeCode}]; ok {
		return v, nil
	}
	if base, ok := armKinematicsBase[modelName]; ok {
		return base, nil
	}
	return kinematicsArtifact{}, fmt.Errorf("no kinematics artifact for xarm model %s", modelName)
}

var gripperKinematicsBase = map[string]kinematicsArtifact{
	ModelNameGripper:           {urdfBasename: "xarm_gripper"},
	ModelNameGripperLite:       {urdfBasename: "uflite_gripper"},
	ModelNameVacuumGripper:     {urdfBasename: "vacuum_gripper"},
	ModelNameVacuumGripperLite: {urdfBasename: "lite_vacuum_gripper"},
}

func resolveGripperKinematicsArtifact(modelName string) (kinematicsArtifact, error) {
	if base, ok := gripperKinematicsBase[modelName]; ok {
		return base, nil
	}
	return kinematicsArtifact{}, fmt.Errorf("no kinematics artifact for gripper model %s", modelName)
}

const gripperDefaultMeshDecimationRatio = 0.1

// loadGripperModel parses the gripper URDF. Nil ratio → default.
func loadGripperModel(modelName string, meshDecimationRatio *float64, logger logging.Logger) (referenceframe.Model, error) {
	artifact, err := resolveGripperKinematicsArtifact(modelName)
	if err != nil {
		return nil, err
	}
	ratio := gripperDefaultMeshDecimationRatio
	if meshDecimationRatio != nil {
		ratio = *meshDecimationRatio
	}
	return makeModelFrameFromURDF(artifact.urdfBasename, modelName, []float64{ratio}, logger)
}
