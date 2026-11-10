package arm

import (
	"embed"
	"encoding/json"
	"fmt"
	"hash/fnv"
	"io/fs"
	"math"
	"os"
	"path/filepath"
	"strings"
	"sync"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
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
// its meshes read from beside it. Unless mesh is set, each collision mesh is replaced by the box
// that bounds it in the link's frame, which keeps planning as cheap as it was with the hand
// authored boxes and the served model small, while the mesh is there for a config that asks.
func loadKinematics(file, resourceName string, mesh bool) (*referenceframe.SimpleModel, error) {
	path, err := kinematicsPath(file)
	if err != nil {
		return nil, err
	}
	pb, err := referenceframe.ReadKinematicModelFile(path)
	if err != nil {
		return nil, err
	}
	if !mesh {
		if err := boxCollisions(pb); err != nil {
			return nil, fmt.Errorf("%s: %w", path, err)
		}
	}
	model, err := referenceframe.ModelFromProto(pb, resourceName)
	if err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	sm, ok := model.(*referenceframe.SimpleModel)
	if !ok {
		return nil, fmt.Errorf("%s loaded as a %T, which cannot carry limits", path, model)
	}
	return sm, nil
}

// boxCollisions replaces every collision mesh in the model with the box that bounds its
// vertices, placed where the mesh was and labelled as it was. A geometry that is not a mesh is
// left alone.
func boxCollisions(pb *commonpb.KinematicModel) error {
	box := func(g *commonpb.Geometry) (*commonpb.Geometry, error) {
		if g.GetMesh() == nil {
			return g, nil
		}
		mesh, err := spatialmath.NewMeshFromProto(spatialmath.NewPoseFromProtobuf(g.GetCenter()), g.GetMesh(), g.GetLabel())
		if err != nil {
			return nil, err
		}
		lo := r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
		hi := lo.Mul(-1)
		for _, tri := range mesh.Triangles() {
			for _, pt := range tri.Points() {
				lo = r3.Vector{X: math.Min(lo.X, pt.X), Y: math.Min(lo.Y, pt.Y), Z: math.Min(lo.Z, pt.Z)}
				hi = r3.Vector{X: math.Max(hi.X, pt.X), Y: math.Max(hi.Y, pt.Y), Z: math.Max(hi.Z, pt.Z)}
			}
		}
		center := spatialmath.NewPoseFromPoint(lo.Add(hi).Mul(0.5))
		bounds, err := spatialmath.NewBox(spatialmath.Compose(mesh.Pose(), center), hi.Sub(lo), g.GetLabel())
		if err != nil {
			return nil, err
		}
		return bounds.ToProtobuf(), nil
	}
	for _, link := range pb.GetLinks() {
		for i, g := range link.GetCollision() {
			boxed, err := box(g)
			if err != nil {
				return fmt.Errorf("link %q collision %d: %w", link.GetId(), i, err)
			}
			link.Collision[i] = boxed
		}
	}
	for _, joint := range pb.GetJoints() {
		if joint.GetGeometry() == nil {
			continue
		}
		boxed, err := box(joint.GetGeometry())
		if err != nil {
			return fmt.Errorf("joint %q geometry: %w", joint.GetId(), err)
		}
		joint.Geometry = boxed
	}
	return nil
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

// wantsMesh reads the collision_meshes choice. use_urdfs used to make the same choice, so a
// config that still sets it and not collision_meshes gets what it asked for rather than a silent
// fall back to the boxes, and a warning either way. The decimation ratios are gone with the
// meshes shipped decimated, so those keys only warn.
func wantsMesh(conf resource.Config, collisionMeshes bool, logger logging.Logger) bool {
	for _, key := range []string{"mesh_decimation_ratio", "mesh_decimation_ratios"} {
		if conf.Attributes.Has(key) {
			logger.Warnf("%s no longer does anything, the meshes ship decimated", key)
		}
	}
	if conf.Attributes.Has("use_urdfs") {
		logger.Warn("use_urdfs is now collision_meshes, set that instead")
		if !conf.Attributes.Has("collision_meshes") {
			return conf.Attributes.Bool("use_urdfs", false)
		}
	}
	return collisionMeshes
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

// loadGripperModel reads the gripper's shipped model file, named for the gripper model, with
// its mesh.
func loadGripperModel(modelName string) (referenceframe.Model, error) {
	artifact, err := resolveGripperKinematicsArtifact(modelName)
	if err != nil {
		return nil, err
	}
	return loadKinematics(artifact.file, modelName, true)
}
