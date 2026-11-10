package arm

import (
	"embed"
	"encoding/json"
	"fmt"
	"hash/fnv"
	"io/fs"
	"os"
	"path/filepath"
	"strings"
	"sync"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// kinematicsArtifact names the kinematic model file an (arm model, hardware variant) pair ships.
// The files live under arm/kinematics with their meshes beside them and are edited in place.
type kinematicsArtifact struct {
	// file is the base name of the model file
	file string
	// variant is a short label used only in logs
	variant string
}

type armVariantKey struct {
	modelName   string
	armTypeCode int
}

var armKinematicsBase = map[string]kinematicsArtifact{
	ModelName6DOF: {file: "xarm6"},
	ModelName7DOF: {file: "xarm7"},
	ModelNameLite: {file: "lite6"},
	ModelName850:  {file: "uf850"},
}

// armTypeCode1305 identifies the xArm6 1305 wrist variant (SN "XI1305…").
const armTypeCode1305 = 1305

// armKinematicsVariants overrides the base entry when the detected armTypeCode matches. The
// xArm6 and its 1305 wrist share frames, limits and visuals and differ in collision meshes.
var armKinematicsVariants = map[armVariantKey]kinematicsArtifact{
	{ModelName6DOF, armTypeCode1305}: {file: "xarm6_1305", variant: "1305"},
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
	ModelNameGripper:           {file: "xarm_gripper"},
	ModelNameGripperLite:       {file: "uflite_gripper"},
	ModelNameVacuumGripper:     {file: "vacuum_gripper"},
	ModelNameVacuumGripperLite: {file: "lite_vacuum_gripper"},
}

func resolveGripperKinematicsArtifact(modelName string) (kinematicsArtifact, error) {
	if base, ok := gripperKinematicsBase[modelName]; ok {
		return base, nil
	}
	return kinematicsArtifact{}, fmt.Errorf("no kinematics artifact for gripper model %s", modelName)
}

// kinematicsFiles is the model files and their meshes, built into the binary so a module started
// straight from the build directory finds them without a module root.
//
//go:embed kinematics
var kinematicsFiles embed.FS

// kinematicsDir unpacks the embedded files once per process, since RDK reads a model file and the
// meshes beside it from disk. The directory is named for the content, so a restart reuses it and
// a new build gets its own.
var kinematicsDir = sync.OnceValues(unpackKinematics)

func unpackKinematics() (string, error) {
	const root = "kinematics"
	digest := fnv.New64a()
	var paths []string
	err := fs.WalkDir(kinematicsFiles, root, func(p string, d fs.DirEntry, err error) error {
		if err != nil || d.IsDir() {
			return err
		}
		data, err := kinematicsFiles.ReadFile(p)
		if err != nil {
			return err
		}
		paths = append(paths, p)
		// a hash never fails to take bytes
		_, _ = digest.Write([]byte(p))
		_, _ = digest.Write(data)
		return nil
	})
	if err != nil {
		return "", err
	}
	dir := filepath.Join(os.TempDir(), fmt.Sprintf("viam-xarm-kinematics-%016x", digest.Sum64()))
	done := filepath.Join(dir, ".complete")
	if _, err := os.Stat(done); err == nil {
		return dir, nil
	}
	// written beside the final name and renamed into place, so a half written directory is never
	// taken for a complete one by another process starting at the same time
	tmp, err := os.MkdirTemp(os.TempDir(), "viam-xarm-kinematics-")
	if err != nil {
		return "", err
	}
	for _, p := range paths {
		data, err := kinematicsFiles.ReadFile(p)
		if err != nil {
			return "", err
		}
		target := filepath.Join(tmp, filepath.FromSlash(strings.TrimPrefix(p, root+"/")))
		if err := os.MkdirAll(filepath.Dir(target), 0o700); err != nil {
			return "", err
		}
		if err := os.WriteFile(target, data, 0o600); err != nil {
			return "", err
		}
	}
	if err := os.WriteFile(filepath.Join(tmp, ".complete"), nil, 0o600); err != nil {
		return "", err
	}
	if err := os.Rename(tmp, dir); err != nil {
		if _, statErr := os.Stat(done); statErr == nil {
			return dir, os.RemoveAll(tmp)
		}
		return "", err
	}
	return dir, nil
}

// kinematicsPath locates a model file in the unpacked directory.
func kinematicsPath(file string) (string, error) {
	dir, err := kinematicsDir()
	if err != nil {
		return "", fmt.Errorf("unpacking the kinematic model files: %w", err)
	}
	return filepath.Join(dir, file+".json"), nil
}

// loadKinematics reads a shipped kinematic model file as a model named for the resource, with
// its meshes read from beside it.
func loadKinematics(file, resourceName string) (*referenceframe.SimpleModel, error) {
	path, err := kinematicsPath(file)
	if err != nil {
		return nil, err
	}
	model, err := referenceframe.ParseModelJSONFile(path, resourceName)
	if err != nil {
		return nil, err
	}
	sm, ok := model.(*referenceframe.SimpleModel)
	if !ok {
		return nil, fmt.Errorf("%s loaded as a %T, which cannot carry limits", path, model)
	}
	return sm, nil
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
	cfg.OriginalFile = &referenceframe.ModelFile{Bytes: raw, Extension: kinematicsSourceJSON}

	return cfg.ParseConfig(name)
}

// warnStaleKeys logs once for each attribute the config still sets that the module no longer
// reads, since the server drops a key the config struct does not have without a word.
func warnStaleKeys(conf resource.Config, logger logging.Logger, keys ...string) {
	for _, key := range keys {
		if conf.Attributes.Has(key) {
			logger.Warnf("%s no longer does anything, the model always carries its collision meshes", key)
		}
	}
}

// gripperWantsMesh reads the gripper's collision geometry choice. use_urdfs used to make the
// same choice, so a config that still sets it and nothing else gets the mesh rather than a
// silent fall back to the boxes, and a warning either way.
func gripperWantsMesh(conf resource.Config, newConf *GripperConfig, logger logging.Logger) bool {
	if conf.Attributes.Has("mesh_decimation_ratio") {
		logger.Warn("mesh_decimation_ratio no longer does anything, the mesh ships decimated")
	}
	if conf.Attributes.Has("use_urdfs") {
		logger.Warnf("use_urdfs is now %s, set that instead", `"collision_geometry": "mesh"`)
		if newConf.CollisionGeometry == "" && conf.Attributes.Bool("use_urdfs", false) {
			return true
		}
	}
	return newConf.CollisionGeometry == collisionGeometryMesh
}

// newGripperKinematics builds the model a gripper reports to the frame system, the shipped
// collision mesh when mesh is set and otherwise a model carrying the hand authored bounding boxes
// that boxGeoms produces.
func newGripperKinematics(
	modelName string,
	mesh bool,
	boxGeoms func() ([]spatialmath.Geometry, error),
) (referenceframe.Model, error) {
	if mesh {
		return loadGripperModel(modelName)
	}
	geoms, err := boxGeoms()
	if err != nil {
		return nil, err
	}
	return makeGeometryModel(modelName, geoms)
}

// loadGripperModel reads the gripper's shipped model file, named for the gripper model.
func loadGripperModel(modelName string) (referenceframe.Model, error) {
	artifact, err := resolveGripperKinematicsArtifact(modelName)
	if err != nil {
		return nil, err
	}
	return loadKinematics(artifact.file, modelName)
}
