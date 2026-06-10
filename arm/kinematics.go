package arm

import (
	"fmt"

	"go.viam.com/rdk/referenceframe"
)

// kinematicsArtifact identifies the kinematics files to use for an
// (arm model, hardware variant) combination.
type kinematicsArtifact struct {
	json         []byte
	urdfBasename string
	// variant is a short label used only in logs (empty string for the
	// base variant of a model).
	variant string
}

type armVariantKey struct {
	modelName   string
	armTypeCode int
}

// armKinematicsBase is keyed by the resource model name. Used when no
// variant lookup matches.
var armKinematicsBase = map[string]kinematicsArtifact{
	ModelName6DOF: {json: xArm6modeljson, urdfBasename: "xarm6"},
	ModelName7DOF: {json: xArm7modeljson, urdfBasename: "xarm7"},
	ModelNameLite: {json: lite6modeljson, urdfBasename: "lite6"},
	ModelName850:  {json: xArm850modeljson, urdfBasename: "uf850"},
}

// armKinematicsVariants overrides the base entry when the detected
// hardware reports a known armTypeCode. xarm6 and xarm6_1305 share the
// upstream kinematics chain (same joint origins and limits per
// xarm6_default_kinematics.yaml) — only collision meshes differ, so the
// variant routes to a distinct URDF but reuses the base JSON.
var armKinematicsVariants = map[armVariantKey]kinematicsArtifact{
	{ModelName6DOF, 1305}: {json: xArm6modeljson, urdfBasename: "xarm6_1305", variant: "1305"},
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

// gripperKinematicsBase is keyed by the gripper resource model name. The
// "vacuum_gripper" and "vacuum_gripper_lite" model names are defined in
// vacuum_gripper.go; we keep them as string literals here to avoid pulling
// the Model literals into the registry.
var gripperKinematicsBase = map[string]kinematicsArtifact{
	ModelNameGripper:      {urdfBasename: "xarm_gripper"},
	ModelNameGripperLite:  {urdfBasename: "uflite_gripper"},
	"vacuum_gripper":      {urdfBasename: "vacuum_gripper"},
	"vacuum_gripper_lite": {urdfBasename: "lite_vacuum_gripper"},
}

func resolveGripperKinematicsArtifact(modelName string) (kinematicsArtifact, error) {
	if base, ok := gripperKinematicsBase[modelName]; ok {
		return base, nil
	}
	return kinematicsArtifact{}, fmt.Errorf("no kinematics artifact for gripper model %s", modelName)
}

func loadGripperModel(modelName string) (referenceframe.Model, error) {
	artifact, err := resolveGripperKinematicsArtifact(modelName)
	if err != nil {
		return nil, err
	}
	return makeModelFrameFromURDF(artifact.urdfBasename, modelName, nil)
}
