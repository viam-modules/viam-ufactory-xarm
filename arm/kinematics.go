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
	"google.golang.org/protobuf/proto"
)

// kinematicsArtifact names the kinematic model file an (arm model, hardware variant) pair ships.
// The files live under arm/kinematics, point at the manufacturer's STL meshes under arm/meshes
// and the viewer's GLB files under arm/3d_models, and are edited in place.
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

// kinematicsFiles is the model files and the meshes they point at, built into the binary so a
// module started straight from the build directory finds them without a module root.
//
//go:embed kinematics meshes 3d_models
var kinematicsFiles embed.FS

var (
	kinematicsDirMu   sync.Mutex
	kinematicsDirPath string
)

// kinematicsDir unpacks the embedded files once per process, since RDK reads a model file and the
// meshes beside it from disk. Only success is kept, and it is checked again on every call, so a
// reconfigure after the disk fills or a cleaner takes the directory unpacks again rather than
// failing for the life of the process.
func kinematicsDir() (string, error) {
	kinematicsDirMu.Lock()
	defer kinematicsDirMu.Unlock()
	if kinematicsDirPath == "" || !unpacked(kinematicsDirPath) {
		dir, err := unpackKinematics()
		if err != nil {
			return "", err
		}
		kinematicsDirPath = dir
	}
	return kinematicsDirPath, nil
}

// unpackRoot is where the files go, the module's own data directory when the server gives one,
// which no temp cleaner touches, and the temp directory otherwise.
func unpackRoot() string {
	if root := os.Getenv("VIAM_MODULE_DATA"); root != "" {
		return root
	}
	return os.TempDir()
}

// unpackKinematics writes the embedded files under the unpack root, named for their content
// and the user, so a restart reuses them, a new build gets its own and another user never finds
// a directory it cannot read. The files are written beside the final name and renamed into
// place, so a half written directory is never taken for a complete one by another process
// starting at the same time, and a leftover that lost its marker to a temp cleaner is replaced.
func unpackKinematics() (string, error) {
	files := map[string][]byte{}
	digest := fnv.New64a()
	err := fs.WalkDir(kinematicsFiles, ".", func(p string, d fs.DirEntry, err error) error {
		if err != nil || d.IsDir() {
			return err
		}
		data, err := kinematicsFiles.ReadFile(p)
		if err != nil {
			return err
		}
		files[p] = data
		// a hash never fails to take bytes, and the walk is in name order so the digest is stable
		_, _ = digest.Write([]byte(p))
		_, _ = digest.Write(data)
		return nil
	})
	if err != nil {
		return "", err
	}
	root := unpackRoot()
	dir := filepath.Join(root, fmt.Sprintf("viam-xarm-kinematics-%d-%016x", os.Getuid(), digest.Sum64()))
	if unpacked(dir) {
		return dir, nil
	}
	if err := os.MkdirAll(root, 0o700); err != nil {
		return "", err
	}
	tmp, err := os.MkdirTemp(root, "viam-xarm-kinematics-")
	if err != nil {
		return "", err
	}
	keep := false
	defer func() {
		if !keep {
			// a copy that is not going to be used, and nothing to do if it stays
			os.RemoveAll(tmp) //nolint:errcheck,gosec
		}
	}()
	for rel, data := range files {
		target := filepath.Join(tmp, filepath.FromSlash(rel))
		if err := os.MkdirAll(filepath.Dir(target), 0o700); err != nil {
			return "", err
		}
		if err := os.WriteFile(target, data, 0o600); err != nil {
			return "", err
		}
	}
	if err := os.WriteFile(filepath.Join(tmp, unpackedMarker), nil, 0o600); err != nil {
		return "", err
	}
	if err := os.Rename(tmp, dir); err == nil || unpacked(dir) {
		return dir, nil
	}
	// a leftover that is not complete, so take its place
	if err := os.RemoveAll(dir); err != nil {
		return "", err
	}
	if err := os.Rename(tmp, dir); err != nil {
		// something keeps winning the name, so this process uses its own copy
		keep = true
		return tmp, nil
	}
	return dir, nil
}

// unpackedMarker is the last file an unpack writes, so its presence says the rest is there.
const unpackedMarker = ".complete"

// unpacked reports whether dir holds a complete unpack this process can read.
func unpacked(dir string) bool {
	//nolint:gosec
	_, err := os.ReadFile(filepath.Join(dir, unpackedMarker))
	return err == nil
}

// kinematicsPath locates a model file in the unpacked directory.
func kinematicsPath(file string) (string, error) {
	dir, err := kinematicsDir()
	if err != nil {
		return "", fmt.Errorf("unpacking the kinematic model files: %w", err)
	}
	return filepath.Join(dir, "kinematics", file+".json"), nil
}

// loadKinematics reads a shipped kinematic model file as a model named for the resource, with
// its meshes read from beside it, after applying each edit to the message in turn.
func loadKinematics(file, resourceName string, edits ...func(*commonpb.KinematicModel) error) (*referenceframe.SimpleModel, error) {
	path, err := kinematicsPath(file)
	if err != nil {
		return nil, err
	}
	pb, err := referenceframe.ReadKinematicModelFile(path)
	if err != nil {
		return nil, err
	}
	for _, edit := range edits {
		if err := edit(pb); err != nil {
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

// defaultMeshDecimationRatio is the share of a collision mesh's triangles kept when the config
// does not say, the ratio the module has always applied.
const defaultMeshDecimationRatio = 0.1

// minTriangles is the fewest triangles the decimator will produce, a box.
const minTriangles = 12

// decimateCollisions replaces each collision mesh with a conservative hull of at most the given
// share of its triangles, the i-th ratio for the i-th link in file order, whether or not that
// link has a mesh, and the default past the end of the list or where the list says 0. A ratio
// of 1 keeps every triangle. The hull contains the mesh, so planning against it is never less
// safe, and far cheaper.
func decimateCollisions(ratios []float64) func(*commonpb.KinematicModel) error {
	return func(pb *commonpb.KinematicModel) error {
		if len(ratios) > len(pb.GetLinks()) {
			return fmt.Errorf("%d mesh decimation ratios for a model with %d links", len(ratios), len(pb.GetLinks()))
		}
		return decimateLinks(pb, func(i int) float64 {
			if i < len(ratios) && ratios[i] > 0 {
				return ratios[i]
			}
			return defaultMeshDecimationRatio
		})
	}
}

// decimateAllCollisions is decimateCollisions with one ratio for every link, the default at 0.
func decimateAllCollisions(ratio float64) func(*commonpb.KinematicModel) error {
	if ratio <= 0 {
		ratio = defaultMeshDecimationRatio
	}
	return func(pb *commonpb.KinematicModel) error {
		return decimateLinks(pb, func(int) float64 { return ratio })
	}
}

func decimateLinks(pb *commonpb.KinematicModel, ratioFor func(link int) float64) error {
	for i, link := range pb.GetLinks() {
		for j, g := range link.GetCollision() {
			if g.GetMesh() == nil {
				continue
			}
			if err := decimateMesh(g.GetMesh(), ratioFor(i)); err != nil {
				return fmt.Errorf("link %q collision %d: %w", link.GetId(), j, err)
			}
		}
	}
	return nil
}

// decimateMesh leaves the mesh as PLY whatever happens, since rdk labels every mesh it serves
// through GetGeometries as PLY whatever bytes it holds, and the shipped files are STL.
func decimateMesh(mesh *commonpb.Mesh, ratio float64) error {
	m, err := spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(), mesh, "")
	if err != nil {
		return err
	}
	before := len(m.Triangles())
	if target := max(int(math.Ceil(ratio*float64(before))), minTriangles); ratio < 1 && target < before {
		if m, err = m.ConservativeDecimate(target); err != nil {
			return err
		}
	}
	mesh.Mesh = m.TrianglesToPLYBytes(false)
	mesh.ContentType = "ply"
	// the bytes no longer match the file the path names, so the path says what they are
	if source := mesh.GetOriginalFileSourcePathDiagnostic(); source != "" {
		mesh.OriginalFileSourcePathDiagnostic = proto.String(strings.TrimSuffix(source, filepath.Ext(source)) + ".ply")
	}
	return nil
}

// validateMeshDecimationRatios checks each ratio is in [0, 1], with 0 standing for the default
// and 1 keeping every triangle.
func validateMeshDecimationRatios(ratios []float64) error {
	for i, r := range ratios {
		if r < 0 || r > 1 || math.IsNaN(r) {
			return fmt.Errorf("mesh decimation ratio %d must be in [0, 1], got %v", i, r)
		}
	}
	return nil
}

// boxCollisions replaces every collision mesh in the model with the box that bounds its
// vertices, placed where the mesh was and labelled as it was, which keeps planning as cheap as
// it was with the hand authored boxes and the served model small. A geometry that is not a mesh
// is left alone.
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
// fall back to the boxes, and a warning either way.
func wantsMesh(conf resource.Config, collisionMeshes bool, logger logging.Logger) bool {
	if conf.Attributes.Has("use_urdfs") {
		logger.Warn("use_urdfs is now collision_meshes, set that instead")
		if !conf.Attributes.Has("collision_meshes") {
			return conf.Attributes.Bool("use_urdfs", false)
		}
	}
	return collisionMeshes
}

// newGripperKinematics builds the model a gripper reports to the frame system, the shipped
// collision mesh decimated at ratio when mesh is set, with each edit applied after, and
// otherwise a model carrying the hand authored bounding boxes that boxGeoms produces. A nil
// ratio is the default.
func newGripperKinematics(
	modelName string,
	mesh bool,
	ratio *float64,
	boxGeoms func() ([]spatialmath.Geometry, error),
	edits ...func(*commonpb.KinematicModel) error,
) (referenceframe.Model, error) {
	if mesh {
		r := defaultMeshDecimationRatio
		if ratio != nil {
			r = *ratio
		}
		return loadGripperModel(modelName, append([]func(*commonpb.KinematicModel) error{decimateAllCollisions(r)}, edits...)...)
	}
	geoms, err := boxGeoms()
	if err != nil {
		return nil, err
	}
	return makeGeometryModel(modelName, geoms)
}

// loadGripperModel reads the gripper's shipped model file, named for the gripper model, with
// each edit applied.
func loadGripperModel(modelName string, edits ...func(*commonpb.KinematicModel) error) (referenceframe.Model, error) {
	artifact, err := resolveGripperKinematicsArtifact(modelName)
	if err != nil {
		return nil, err
	}
	return loadKinematics(artifact.file, modelName, edits...)
}

// vacuumTube appends the suction tube, whose length is set per deployment, to a vacuum
// gripper's model as a thin box past the TCP, so the model the frame system plans with carries
// it and not only the geometries the gripper reports.
func vacuumTube(lengthMM float64) func(*commonpb.KinematicModel) error {
	return func(pb *commonpb.KinematicModel) error {
		if lengthMM <= 0 {
			return nil
		}
		tip, err := leafLink(pb)
		if err != nil {
			return err
		}
		tube, err := spatialmath.NewBox(
			spatialmath.NewPoseFromPoint(r3.Vector{Z: lengthMM / 2}),
			r3.Vector{X: 5, Y: 5, Z: lengthMM}, vacuumTubeLabel)
		if err != nil {
			return err
		}
		pb.Links = append(pb.Links, &commonpb.KinematicLink{
			Id:        vacuumTubeLabel,
			Parent:    tip,
			Pose:      &commonpb.LinkPose{},
			Collision: []*commonpb.Geometry{tube.ToProtobuf()},
		})
		return nil
	}
}

const vacuumTubeLabel = "vacuum-gripper-tube"

// leafLink names the one link nothing hangs off, the model's TCP.
func leafLink(pb *commonpb.KinematicModel) (string, error) {
	parents := map[string]bool{}
	for _, l := range pb.GetLinks() {
		parents[l.GetParent()] = true
	}
	for _, j := range pb.GetJoints() {
		parents[j.GetParent()] = true
	}
	var leaves []string
	for _, l := range pb.GetLinks() {
		if !parents[l.GetId()] {
			leaves = append(leaves, l.GetId())
		}
	}
	if len(leaves) != 1 {
		return "", fmt.Errorf("model %q has %d leaf links, want one", pb.GetName(), len(leaves))
	}
	return leaves[0], nil
}
