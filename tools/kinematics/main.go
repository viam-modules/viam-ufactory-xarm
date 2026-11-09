// Command kinematics writes the kinematic model files the module ships from the sources checked
// in beside them. An arm's frames, joint limits and link names come from its SVA document, its
// collision meshes from the URDF the manufacturer publishes, and its visuals from the GLB files
// the viewer shows, so one file per arm carries all three. A gripper comes from its URDF alone.
// Run it from the repository root, go run ./tools/kinematics, after changing any of the sources.
package main

import (
	"cmp"
	"flag"
	"fmt"
	"io"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"slices"
	"strings"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

// decimationRatio is the share of a collision mesh's triangles the shipped file keeps. The module
// applied the same ratio at startup when it read the URDF itself, so planning sees the meshes it did.
const decimationRatio = 0.1

// minTriangles is the fewest triangles the decimator will produce, a box.
const minTriangles = 12

type armSource struct {
	// name is the model's name and the base of the file written
	name string
	// sva carries the frames, the joint limits and the link names
	sva string
	// urdf carries the collision meshes
	urdf string
	// visuals is a directory of GLB files named for the links they show, or empty
	visuals string
}

// arms are the files each arm is built from. The xArm6 and its 1305 wrist variant share the SVA
// document and the visuals and differ in collision meshes only.
var arms = []armSource{
	{name: "xarm6", sva: "arm/xarm6_kinematics.json", urdf: "arm/xarm6.urdf", visuals: "arm/3d_models/xArm6"},
	{name: "xarm6_1305", sva: "arm/xarm6_kinematics.json", urdf: "arm/xarm6_1305.urdf", visuals: "arm/3d_models/xArm6"},
	{name: "xarm7", sva: "arm/xarm7_kinematics.json", urdf: "arm/xarm7.urdf"},
	{name: "lite6", sva: "arm/lite6_kinematics.json", urdf: "arm/lite6.urdf", visuals: "arm/3d_models/lite6"},
	{name: "uf850", sva: "arm/xarm850_kinematics.json", urdf: "arm/uf850.urdf"},
}

var grippers = []string{"xarm_gripper", "uflite_gripper", "vacuum_gripper", "lite_vacuum_gripper"}

func main() {
	out := flag.String("out", "arm/kinematics", "the directory to write the model files into")
	flag.Parse()
	if err := run(*out, os.Stdout); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run(outDir string, w io.Writer) error {
	for _, a := range arms {
		if err := writeArm(a, outDir, w); err != nil {
			return fmt.Errorf("%s: %w", a.name, err)
		}
	}
	for _, g := range grippers {
		if err := writeGripper(g, outDir, w); err != nil {
			return fmt.Errorf("%s: %w", g, err)
		}
	}
	return nil
}

// writeArm puts the URDF's collision meshes on the SVA model's links and writes the result. The
// two describe the same chain but place their link frames differently, the URDF rotates them
// and the SVA does not, so each mesh is re-expressed in the frame of the SVA link it moves with.
func writeArm(src armSource, outDir string, w io.Writer) error {
	base, err := simpleModel(referenceframe.ParseModelJSONFile(src.sva, src.name))
	if err != nil {
		return err
	}
	urdf, err := readURDF(src.urdf, w)
	if err != nil {
		return err
	}
	cfg := base.ModelConfig()
	baseSegments, err := segments(cfg)
	if err != nil {
		return fmt.Errorf("%s: %w", src.sva, err)
	}
	urdfSegments, err := segments(urdf.ModelConfig())
	if err != nil {
		return fmt.Errorf("%s: %w", src.urdf, err)
	}
	if len(baseSegments) != len(urdfSegments) {
		return fmt.Errorf("%s has %d rigid segments and %s has %d", src.sva, len(baseSegments), src.urdf, len(urdfSegments))
	}
	basePoses, err := linkPoses(base)
	if err != nil {
		return err
	}
	urdfPoses, err := linkPoses(urdf)
	if err != nil {
		return err
	}

	links := make([]referenceframe.LinkConfig, len(cfg.Links))
	copy(links, cfg.Links)
	index := map[string]int{}
	for i, l := range links {
		index[l.ID] = i
	}
	urdfLinks := map[string]referenceframe.LinkConfig{}
	for _, l := range urdf.ModelConfig().Links {
		urdfLinks[l.ID] = l
	}
	for i, segment := range baseSegments {
		// the last link of a segment is the one the viewer's GLB is named for, so the mesh goes there too
		target := segment[len(segment)-1]
		var meshes []spatialmath.GeometryConfig
		for _, id := range urdfSegments[i] {
			for _, gc := range collisions(urdfLinks[id]) {
				geometry, err := gc.ParseConfig()
				if err != nil {
					return fmt.Errorf("link %q of %s: %w", id, src.urdf, err)
				}
				// the geometry sits at its URDF link's parent, so carry it through world to the target's
				moved := geometry.Transform(spatialmath.Compose(spatialmath.PoseInverse(basePoses[target]), urdfPoses[id]))
				mc, err := spatialmath.NewGeometryConfig(moved)
				if err != nil {
					return fmt.Errorf("link %q of %s: %w", id, src.urdf, err)
				}
				mc.Label = target
				if len(meshes) > 0 {
					mc.Label = fmt.Sprintf("%s_%d", target, len(meshes))
				}
				if mc.Type == spatialmath.MeshType {
					mc.MeshFilePath = mc.Label + "." + mc.MeshContentType
				}
				meshes = append(meshes, *mc)
			}
		}
		if len(meshes) == 0 {
			// nothing to put in its place, so the segment keeps whatever the SVA gave it
			if len(collisions(links[index[target]])) == 0 {
				sayf(w, "  %s: %s has no collision geometry in either source\n", src.name, target)
			}
			continue
		}
		for _, id := range segment {
			links[index[id]].Geometry = nil
			links[index[id]].Geometries = nil
		}
		links[index[target]].Geometry = &meshes[0]
		links[index[target]].Geometries = meshes[1:]
	}

	out := *cfg
	out.Name = src.name
	out.Links = links
	out.OriginalFile = nil
	merged, err := simpleModel(out.ParseConfig(src.name))
	if err != nil {
		return err
	}
	if err := sameMeshes(urdf, merged); err != nil {
		return fmt.Errorf("the meshes did not land where %s has them: %w", src.urdf, err)
	}
	if src.visuals != "" {
		if err := attachVisuals(merged, links, src.visuals); err != nil {
			return err
		}
	}
	return write(merged, base, filepath.Join(outDir, src.name+".json"), w)
}

// writeGripper writes a gripper's URDF as a model file. The URDF is its only source, so the
// frames carry their meshes already and nothing moves.
func writeGripper(name, outDir string, w io.Writer) error {
	model, err := readURDF(filepath.Join("arm", name+".urdf"), w)
	if err != nil {
		return err
	}
	return write(model, model, filepath.Join(outDir, name+".json"), w)
}

// write decimates the model's collision meshes, writes it beside them and reads the file back
// against want, so a file that lost a joint, a limit or a frame fails here rather than on an arm.
func write(model *referenceframe.SimpleModel, want referenceframe.Model, path string, w io.Writer) error {
	pb, err := referenceframe.ModelToProto(model)
	if err != nil {
		return err
	}
	pb.Name = strings.TrimSuffix(filepath.Base(path), ".json")
	if err := decimate(pb, w); err != nil {
		return err
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o750); err != nil {
		return err
	}
	if err := referenceframe.WriteKinematicModelFile(path, pb); err != nil {
		return err
	}
	got, err := referenceframe.ParseModelJSONFile(path, "")
	if err != nil {
		return err
	}
	if err := sameChain(want, got); err != nil {
		return fmt.Errorf("%s does not match its source: %w", path, err)
	}
	collisions, visuals, meshBytes := count(pb)
	sayf(w, "wrote %s: %d joints, %d links, %d collision geometries, %d visuals, %.1f MB of meshes\n",
		path, len(pb.GetJoints()), len(pb.GetLinks()), collisions, visuals, float64(meshBytes)/1e6)
	return nil
}

// sayf prints a progress line. Nothing is left to do when the terminal is gone.
func sayf(w io.Writer, format string, args ...any) {
	//nolint:errcheck
	fmt.Fprintf(w, format, args...)
}

func readURDF(path string, w io.Writer) (*referenceframe.SimpleModel, error) {
	model, skipped, err := referenceframe.ReadURDFModel(path, "")
	for _, reason := range skipped {
		sayf(w, "%s: skipped %s\n", path, reason)
	}
	return simpleModel(model, err)
}

func simpleModel(model referenceframe.Model, err error) (*referenceframe.SimpleModel, error) {
	if err != nil {
		return nil, err
	}
	sm, ok := model.(*referenceframe.SimpleModel)
	if !ok {
		return nil, fmt.Errorf("model %q is a %T, not a simple model", model.Name(), model)
	}
	return sm, nil
}

// segments lists a serial model's links in rigid groups, the links between one moving joint and
// the next, nearest world first. Links on either side of a moving joint are in different groups.
func segments(cfg *referenceframe.ModelConfigJSON) ([][]string, error) {
	children := map[string][]string{}
	isJoint := map[string]bool{}
	// a URDF whose root link has no joint above it comes through with an empty parent
	parentOf := func(parent string) string {
		if parent == "" {
			return referenceframe.World
		}
		return parent
	}
	for _, l := range cfg.Links {
		children[parentOf(l.Parent)] = append(children[parentOf(l.Parent)], l.ID)
	}
	for _, j := range cfg.Joints {
		children[parentOf(j.Parent)] = append(children[parentOf(j.Parent)], j.ID)
		isJoint[j.ID] = true
	}
	var out [][]string
	var current []string
	for node := referenceframe.World; ; {
		next := children[node]
		if len(next) == 0 {
			break
		}
		if len(next) > 1 {
			return nil, fmt.Errorf("%q has %d children, so the model is not a serial chain", node, len(next))
		}
		node = next[0]
		if isJoint[node] {
			out = append(out, current)
			current = nil
			continue
		}
		current = append(current, node)
	}
	return append(out, current), nil
}

// linkPoses gives, for every link, the pose in world of the frame its geometry is placed in,
// with the joints at zero. RDK places a link's geometry at the link's parent, so that is the
// parent's pose, and world itself for a root link.
func linkPoses(model referenceframe.Model) (map[string]spatialmath.Pose, error) {
	const part = "m"
	fs, err := referenceframe.NewFrameSystem("poses", []*referenceframe.FrameSystemPart{{
		FrameConfig: referenceframe.NewLinkInFrame(referenceframe.World, spatialmath.NewZeroPose(), part, nil),
		ModelFrame:  model,
	}}, nil)
	if err != nil {
		return nil, err
	}
	inputs := referenceframe.NewLinearInputs()
	inputs.Put(part, make([]referenceframe.Input, len(model.DoF())))
	poses := map[string]spatialmath.Pose{}
	for _, l := range model.ModelConfig().Links {
		if l.Parent == referenceframe.World || l.Parent == "" {
			poses[l.ID] = spatialmath.NewZeroPose()
			continue
		}
		tf, err := fs.Transform(inputs, referenceframe.NewPoseInFrame(part+":"+l.Parent, spatialmath.NewZeroPose()), referenceframe.World)
		if err != nil {
			return nil, fmt.Errorf("pose of the parent of link %q: %w", l.ID, err)
		}
		pif, ok := tf.(*referenceframe.PoseInFrame)
		if !ok {
			return nil, fmt.Errorf("pose of the parent of link %q came back as a %T", l.ID, tf)
		}
		poses[l.ID] = pif.Pose()
	}
	return poses, nil
}

func collisions(link referenceframe.LinkConfig) []spatialmath.GeometryConfig {
	if link.Geometry == nil {
		return link.Geometries
	}
	return append([]spatialmath.GeometryConfig{*link.Geometry}, link.Geometries...)
}

// attachVisuals puts each GLB in dir on the link it is named for. The viewer attaches a GLB to
// the frame named for its link, and RDK places a link's geometry at the link's parent, so the
// visual carries the link's own transform to land in the same place. A GLB named for a link the
// model does not have is an error, since the viewer would lose a part it shows today.
func attachVisuals(model *referenceframe.SimpleModel, links []referenceframe.LinkConfig, dir string) error {
	entries, err := os.ReadDir(dir)
	if err != nil {
		return err
	}
	zero, err := spatialmath.NewOrientationConfig(spatialmath.NewZeroOrientation())
	if err != nil {
		return err
	}
	for _, e := range entries {
		if filepath.Ext(e.Name()) != ".glb" {
			continue
		}
		link := strings.TrimSuffix(e.Name(), ".glb")
		i := slices.IndexFunc(links, func(l referenceframe.LinkConfig) bool { return l.ID == link })
		if i < 0 {
			return fmt.Errorf("visual %s names a link the model does not have", filepath.Join(dir, e.Name()))
		}
		//nolint:gosec
		data, err := os.ReadFile(filepath.Join(dir, e.Name()))
		if err != nil {
			return err
		}
		visual := spatialmath.GeometryConfig{
			Type:              spatialmath.MeshType,
			MeshData:          data,
			MeshContentType:   "glb",
			MeshFilePath:      e.Name(),
			TranslationOffset: links[i].Translation,
			OrientationOffset: *zero,
			Label:             link,
		}
		if links[i].Orientation != nil {
			visual.OrientationOffset = *links[i].Orientation
		}
		if err := model.SetVisualGeometries(link, []spatialmath.GeometryConfig{visual}); err != nil {
			return fmt.Errorf("visual %s: %w", filepath.Join(dir, e.Name()), err)
		}
	}
	return nil
}

// decimate replaces every collision mesh with a conservative hull of at most decimationRatio of
// its triangles, as PLY since that is what the decimator produces.
func decimate(pb *commonpb.KinematicModel, w io.Writer) error {
	one := func(owner string, mesh *commonpb.Mesh) error {
		if mesh == nil {
			return nil
		}
		m, err := spatialmath.NewMeshFromProto(spatialmath.NewZeroPose(), mesh, "")
		if err != nil {
			return fmt.Errorf("%s: %w", owner, err)
		}
		before := len(m.Triangles())
		target := max(int(math.Ceil(decimationRatio*float64(before))), minTriangles)
		if target >= before {
			return nil
		}
		decimated, err := m.ConservativeDecimate(target)
		if err != nil {
			return fmt.Errorf("%s: %w", owner, err)
		}
		mesh.Mesh = decimated.TrianglesToPLYBytes(false)
		mesh.ContentType = "ply"
		sayf(w, "  %s: %d to %d triangles\n", owner, before, len(decimated.Triangles()))
		return nil
	}
	for _, link := range pb.GetLinks() {
		for i, g := range link.GetCollision() {
			if err := one(fmt.Sprintf("link %q collision %d", link.GetId(), i), g.GetMesh()); err != nil {
				return err
			}
		}
	}
	for _, joint := range pb.GetJoints() {
		if err := one(fmt.Sprintf("joint %q geometry", joint.GetId()), joint.GetGeometry().GetMesh()); err != nil {
			return err
		}
	}
	return nil
}

// sameChain checks that got has want's joints, limits and forward kinematics.
func sameChain(want, got referenceframe.Model) error {
	if len(got.DoF()) != len(want.DoF()) {
		return fmt.Errorf("%d joints, want %d", len(got.DoF()), len(want.DoF()))
	}
	// the file holds degrees, so a limit comes back a few ulps from where it started
	for i, limit := range want.DoF() {
		if !sameLimit(got.DoF()[i], limit) {
			return fmt.Errorf("joint %d has limits %+v, want %+v", i, got.DoF()[i], limit)
		}
	}
	//nolint:gosec
	rng := rand.New(rand.NewSource(1))
	for range 100 {
		q := referenceframe.RandomFrameInputs(want, rng)
		a, err := want.Transform(q)
		if err != nil {
			return err
		}
		b, err := got.Transform(q)
		if err != nil {
			return err
		}
		if !spatialmath.PoseAlmostEqualEps(a, b, 1e-6) {
			return fmt.Errorf("at %v the end is at %v, want %v", q, spatialmath.PoseToProtobuf(b), spatialmath.PoseToProtobuf(a))
		}
	}
	return nil
}

func sameLimit(a, b referenceframe.Limit) bool {
	same := func(x, y *float64) bool {
		if x == nil || y == nil {
			return x == y
		}
		return math.Abs(*x-*y) < 1e-9
	}
	return math.Abs(a.Min-b.Min) < 1e-9 && math.Abs(a.Max-b.Max) < 1e-9 &&
		same(a.MaxVelocity, b.MaxVelocity) && same(a.MaxAcceleration, b.MaxAcceleration)
}

// meshTolerance is how far a mesh may sit from where the URDF has it. The two sources round
// their lengths differently and the lite6 differ on the base height by 0.2 mm, which adds up to
// 1.4 mm at the wrist, while a frame put in the wrong place is off by centimetres or a rotation.
const meshTolerance = 2.0

// sameMeshes checks that the collision meshes of got sit where those of want do at several
// joint positions, by comparing where their vertices land in world. Only meshes count, since
// a link that got no mesh keeps the box its SVA document gave it.
func sameMeshes(want, got referenceframe.Model) error {
	//nolint:gosec
	rng := rand.New(rand.NewSource(2))
	for range 10 {
		q := referenceframe.RandomFrameInputs(want, rng)
		a, err := meshCentroids(want, q)
		if err != nil {
			return err
		}
		b, err := meshCentroids(got, q)
		if err != nil {
			return err
		}
		if len(a) != len(b) {
			return fmt.Errorf("%d meshes, want %d", len(b), len(a))
		}
		for i := range a {
			if a[i].Distance(b[i]) > meshTolerance {
				return fmt.Errorf("at %v a mesh is centred at %v, want %v", q, b[i], a[i])
			}
		}
	}
	return nil
}

// meshCentroids gives the world centroid of every collision mesh's vertices at q, sorted so two
// models can be compared without matching labels.
func meshCentroids(model referenceframe.Model, q []referenceframe.Input) ([]r3.Vector, error) {
	gif, err := model.Geometries(q)
	if err != nil {
		return nil, err
	}
	var out []r3.Vector
	for _, g := range gif.Geometries() {
		mesh, ok := g.(*spatialmath.Mesh)
		if !ok {
			continue
		}
		var sum r3.Vector
		var n float64
		for _, tri := range mesh.Triangles() {
			for _, p := range tri.Points() {
				sum = sum.Add(spatialmath.Compose(mesh.Pose(), spatialmath.NewPoseFromPoint(p)).Point())
				n++
			}
		}
		out = append(out, sum.Mul(1/n))
	}
	slices.SortFunc(out, func(a, b r3.Vector) int {
		return cmp.Or(cmp.Compare(a.X, b.X), cmp.Compare(a.Y, b.Y), cmp.Compare(a.Z, b.Z))
	})
	return out, nil
}

func count(pb *commonpb.KinematicModel) (collisions, visuals, meshBytes int) {
	for _, link := range pb.GetLinks() {
		collisions += len(link.GetCollision())
		visuals += len(link.GetVisual())
		for _, g := range link.GetCollision() {
			meshBytes += len(g.GetMesh().GetMesh())
		}
		for _, g := range link.GetVisual() {
			meshBytes += len(g.GetMesh().GetMesh())
		}
	}
	for _, joint := range pb.GetJoints() {
		if joint.GetGeometry() != nil {
			collisions++
			meshBytes += len(joint.GetGeometry().GetMesh().GetMesh())
		}
	}
	return collisions, visuals, meshBytes
}
