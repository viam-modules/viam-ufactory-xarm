package arm

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strings"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
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
	// outputFrame names the end effector of a URDF with several leaves; empty
	// for a URDF with one.
	outputFrame string
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

// makeGeometryModel builds a zero-DoF kinematics model whose links carry geoms.
// Each geometry gets its own link, chained parent-to-child, because a link
// config holds at most one geometry.
//
// The config is marshalled into OriginalFile before being parsed. RDK forwards a
// model over GetKinematics only when ModelConfig().OriginalFile is set (see
// referenceframe.KinematicModelToProtobuf); without it the response carries no
// kinematics data, the caller reconstructs an empty model, and the gripper lands
// in the frame system with nothing to collide against.
func makeGeometryModel(name string, geoms []spatialmath.Geometry) (referenceframe.Model, error) {
	if len(geoms) == 0 {
		return nil, fmt.Errorf("no geometries to build a kinematics model for %s", name)
	}

	cfg := &referenceframe.ModelConfigJSON{Name: name}
	parent := referenceframe.World
	for _, geom := range geoms {
		frame, err := referenceframe.NewStaticFrameWithGeometry(geom.Label(), spatialmath.NewZeroPose(), geom)
		if err != nil {
			return nil, err
		}
		link, err := referenceframe.NewLinkConfig(frame)
		if err != nil {
			return nil, err
		}
		link.Parent = parent
		parent = geom.Label()
		cfg.Links = append(cfg.Links, *link)
	}

	raw, err := json.Marshal(cfg)
	if err != nil {
		return nil, err
	}
	cfg.OriginalFile = &referenceframe.ModelFile{Bytes: raw, Extension: "json"}

	return cfg.ParseConfig(name)
}

// newGripperKinematics builds the model a gripper reports to the frame system:
// URDF-derived meshes when use_urdfs is set, otherwise a model carrying the
// hand-authored bounding boxes that boxGeoms produces.
func newGripperKinematics(
	modelName string,
	conf *GripperConfig,
	logger logging.Logger,
	boxGeoms func() ([]spatialmath.Geometry, error),
) (referenceframe.Model, error) {
	if conf.UseURDFs {
		return loadGripperModel(modelName, conf.MeshDecimationRatio, logger)
	}
	geoms, err := boxGeoms()
	if err != nil {
		return nil, err
	}
	return makeGeometryModel(modelName, geoms)
}

const gripperDefaultMeshDecimationRatio = 0.1

// loadGripperModel parses the gripper URDF. Nil ratio → default.
func loadGripperModel(modelName string, meshDecimationRatio *float64, logger logging.Logger) (referenceframe.Model, error) {
	artifact, err := resolveGripperKinematicsArtifact(modelName)
	if err != nil {
		return nil, err
	}
	return makeModelFrameFromURDF(artifact.urdfBasename, modelName, []float64{gripperMeshDecimationRatio(meshDecimationRatio)}, logger)
}

func gripperMeshDecimationRatio(meshDecimationRatio *float64) float64 {
	if meshDecimationRatio != nil {
		return *meshDecimationRatio
	}
	return gripperDefaultMeshDecimationRatio
}

// standardGripperFingers is the standard gripper's moving-finger model, used
// when moving_fingers is set. Its fingers branch off the base beside the TCP,
// so it needs an explicit output frame.
var standardGripperFingers = kinematicsArtifact{urdfBasename: "xarm_gripper_fingers", outputFrame: "link_tcp"}

// loadStandardGripperFingerModel parses the moving-finger URDF, with the base
// mesh decimated by meshDecimationRatio. Nil ratio → default.
func loadStandardGripperFingerModel(modelName string, meshDecimationRatio *float64, logger logging.Logger) (referenceframe.Model, error) {
	return makeModelFrameFromURDFWithOutputFrame(standardGripperFingers.urdfBasename, modelName, standardGripperFingers.outputFrame,
		[]float64{gripperMeshDecimationRatio(meshDecimationRatio)}, logger)
}

// makeModelFrameFromURDFWithOutputFrame parses a multi-leaf URDF with
// outputFrame as its end effector, recorded as SVA JSON because a URDF cannot
// carry the output frame over GetKinematics.
func makeModelFrameFromURDFWithOutputFrame(
	urdfBasename, modelName, outputFrame string,
	meshDecimationRatios []float64,
	logger logging.Logger,
) (referenceframe.Model, error) {
	ratios := clampMeshDecimationRatios(meshDecimationRatios, modelName, logger)
	cfg, err := unmarshalURDFFile(urdfBasename, modelName, ratios)
	if err != nil {
		return nil, err
	}
	return parseModelWithOutputFrame(cfg, modelName, outputFrame)
}

// unmarshalURDFFile reads a shipped URDF and its meshes into a model config
// without building the model, so callers can adjust it first.
func unmarshalURDFFile(urdfBasename, modelName string, meshDecimationRatios []float64) (*referenceframe.ModelConfigJSON, error) {
	path := urdfPath(urdfBasename)
	xmlData, err := os.ReadFile(path) //nolint:gosec // path is built from a fixed basename.
	if err != nil {
		return nil, fmt.Errorf("failed to read URDF file: %w", err)
	}
	meshMap, err := loadURDFMeshes(xmlData, filepath.Dir(path))
	if err != nil {
		return nil, err
	}
	return referenceframe.UnmarshalModelXML(xmlData, modelName, meshMap, meshDecimationRatios)
}

// parseModelWithOutputFrame names outputFrame as cfg's end effector and builds
// the model, recording cfg as SVA JSON so it survives GetKinematics.
func parseModelWithOutputFrame(cfg *referenceframe.ModelConfigJSON, modelName, outputFrame string) (referenceframe.Model, error) {
	cfg.OutputFrames = []string{outputFrame}

	cfg.OriginalFile = nil
	raw, err := json.Marshal(cfg)
	if err != nil {
		return nil, err
	}
	cfg.OriginalFile = &referenceframe.ModelFile{Bytes: raw, Extension: "json"}

	return cfg.ParseConfig(modelName)
}

// Link IDs in xarm_gripper_fingers.urdf's parsed config that the box model rewrites.
// RDK turns each fixed joint into a link of the same name, so world_joint and
// joint_tcp are the offsets to the base and to link_tcp.
const (
	gripperURDFRootLink = "world_joint"
	gripperURDFBaseLink = "xarm_gripper_base_link"
	gripperURDFTCPLink  = "joint_tcp"
)

// standardGripperBoxModel is the moving-finger model without use_urdfs: the
// finger URDF with the case box as the body, in the box model's frame (origin
// at the jaw end of the case, mount face at -case length).
func standardGripperBoxModel(modelName, submodel string) (referenceframe.Model, error) {
	// No decimation: the base mesh is parsed only to be replaced.
	cfg, err := unmarshalURDFFile(standardGripperFingers.urdfBasename, modelName, nil)
	if err != nil {
		return nil, err
	}

	caseSize := standardGripperCaseSize(submodel)
	caseBox, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Z: caseSize.Z / 2}), caseSize, standardGripperCaseLabel)
	if err != nil {
		return nil, err
	}
	caseGeom, err := spatialmath.NewGeometryConfig(caseBox)
	if err != nil {
		return nil, err
	}

	found := map[string]bool{}
	for i := range cfg.Links {
		link := &cfg.Links[i]
		switch link.ID {
		case gripperURDFRootLink:
			link.Translation = r3.Vector{Z: -caseSize.Z}
		case gripperURDFBaseLink:
			// The link ID becomes the geometry label; keep the box model's.
			link.ID = standardGripperCaseLabel
			link.Geometry = caseGeom
		case gripperURDFTCPLink:
			link.Translation = r3.Vector{Z: caseSize.Z}
		}
		found[link.ID] = true
		if link.Parent == gripperURDFBaseLink {
			link.Parent = standardGripperCaseLabel
		}
	}
	for _, id := range []string{gripperURDFRootLink, standardGripperCaseLabel, gripperURDFTCPLink} {
		if !found[id] {
			return nil, fmt.Errorf("gripper box model: %s.urdf has no link %q", standardGripperFingers.urdfBasename, id)
		}
	}
	for i := range cfg.Joints {
		if cfg.Joints[i].Parent == gripperURDFBaseLink {
			cfg.Joints[i].Parent = standardGripperCaseLabel
		}
	}

	return parseModelWithOutputFrame(cfg, modelName, standardGripperFingers.outputFrame)
}

var urdfMeshFilenameRE = regexp.MustCompile(`<mesh\s+filename="([^"]+)"`)

// loadURDFMeshes reads every mesh a URDF references, keyed the way
// UnmarshalModelXML looks them up. It mirrors RDK's unexported
// buildMeshMapFromURDF, which ParseModelXMLFile uses.
func loadURDFMeshes(xmlData []byte, urdfDir string) (map[string]*commonpb.Mesh, error) {
	meshMap := map[string]*commonpb.Mesh{}
	for _, m := range urdfMeshFilenameRE.FindAllSubmatch(xmlData, -1) {
		meshPath := normalizeURDFMeshPath(string(m[1]))
		if _, ok := meshMap[meshPath]; ok {
			continue
		}
		var contentType string
		switch strings.ToLower(filepath.Ext(meshPath)) {
		case ".stl":
			contentType = "stl"
		case ".ply":
			contentType = "ply"
		default:
			return nil, fmt.Errorf("unsupported mesh file type (only .ply and .stl supported): %s", meshPath)
		}
		meshBytes, err := os.ReadFile(filepath.Join(urdfDir, meshPath)) //nolint:gosec // paths come from shipped URDFs.
		if err != nil {
			return nil, fmt.Errorf("failed to load mesh file %s: %w", meshPath, err)
		}
		meshMap[meshPath] = &commonpb.Mesh{Mesh: meshBytes, ContentType: contentType}
	}
	return meshMap, nil
}

// normalizeURDFMeshPath strips a package:// URI down to a path relative to the
// URDF, e.g. "package://description/meshes/a.stl" -> "meshes/a.stl", matching
// RDK's own normalisation.
func normalizeURDFMeshPath(meshPath string) string {
	const prefix = "package://"
	if !strings.HasPrefix(meshPath, prefix) {
		return meshPath
	}
	meshPath = strings.TrimPrefix(meshPath, prefix)
	if idx := strings.Index(meshPath, "/"); idx != -1 {
		meshPath = meshPath[idx+1:]
	}
	return meshPath
}
