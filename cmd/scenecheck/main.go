// Command scenecheck asks a machine's motion planner whether its world scene keeps the
// gripper out of the table. It only plans; it never moves the arm.
//
// Checks:
//  1. frames: world poses of the plate, arm, and gripper.
//  2. start: no arm/gripper geometry already touches an obstacle. The planner ignores any
//     pair in collision at the start of a move, so a touching pair silently disables that
//     obstacle for the move.
//  3. sweep: plans the gripper straight down to a column of heights around the table top.
//     Goals below the table top must fail to plan.
//  4. paths: every plan that succeeded is re-checked here, interpolated between waypoints,
//     for arm/gripper geometry hitting an obstacle, and its closest approach to the table
//     is reported.
package main

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"math"
	"os"
	"slices"
	"strings"

	"github.com/go-viper/mapstructure/v2"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
	"google.golang.org/protobuf/encoding/protojson"
)

var (
	addr        = flag.String("addr", "localhost:18080", "machine address; ignored when -cred is set")
	credPath    = flag.String("cred", "", "machine cloud-credentials file (Remote address line + cloud JSON)")
	armName     = flag.String("arm", "arm", "arm component")
	gripperName = flag.String("gripper", "gripper", "gripper component")
	tableName   = flag.String("table", "table", "table obstacle")
	plateName   = flag.String("plate", "xarm6-mounting-plate", "mounting plate component")
	motionName  = flag.String("motion", "builtin", "motion service")
	goalX       = flag.Float64("x", 350, "world x of the sweep column, mm")
	goalY       = flag.Float64("y", 0, "world y of the sweep column, mm")
	heights     = flag.String("heights", "150,60,55,50,45,30,15,5,0,-10,-30", "sweep heights relative to the table top, mm")
	fixedLinks  = flag.String("fixed-links", "base,base_top", "arm links that only turn about the base axis; their contact with the mount is expected")
	substeps    = flag.Int("substeps", 20, "interpolation steps between trajectory waypoints")
)

func main() {
	flag.Parse()
	failures, err := run(context.Background())
	if err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(2)
	}
	if failures > 0 {
		printf("\nFAIL: %d check(s) failed\n", failures)
		os.Exit(1)
	}
	printLine("\nPASS")
}

func run(ctx context.Context) (int, error) {
	logger := logging.NewLogger("scenecheck")
	logger.SetLevel(logging.WARN)
	r, err := connect(ctx, logger)
	if err != nil {
		return 0, err
	}
	defer func() { _ = r.Close(ctx) }() //nolint:errcheck

	fsCfg, err := r.FrameSystemConfig(ctx)
	if err != nil {
		return 0, err
	}
	fs, err := referenceframe.NewFrameSystem("scenecheck", fsCfg.Parts, nil)
	if err != nil {
		return 0, err
	}
	start, err := r.CurrentInputs(ctx)
	if err != nil {
		return 0, err
	}
	moving, obstacles := classifyFrames(fs)
	printf("moving frames:   %s\nobstacle frames: %s\n", strings.Join(moving, ", "), strings.Join(obstacles, ", "))

	failures := 0

	printLine("\n== frames (world)")
	for _, f := range []string{*plateName + "_origin", *plateName, *armName + "_origin", *gripperName} {
		p, err := fs.Transform(start.ToLinearInputs(), referenceframe.NewPoseInFrame(f, spatialmath.NewZeroPose()), referenceframe.World)
		if err != nil {
			printf("  %-30s ERROR %v\n", f, err)
			failures++
			continue
		}
		pose := p.(*referenceframe.PoseInFrame).Pose()
		printf("  %-30s %s\n", f, fmtPose(pose))
	}

	geoms, err := referenceframe.FrameSystemGeometries(fs, start)
	if err != nil {
		return 0, err
	}
	tableTop, err := topOf(geoms, *tableName)
	if err != nil {
		return 0, err
	}
	printf("  table top z = %.1f\n", tableTop)
	if plateBottom, err := bottomOf(geoms, *plateName); err == nil {
		gap := plateBottom - tableTop
		printf("  plate bottom z = %.1f (gap above table %.1f)\n", plateBottom, gap)
		if math.Abs(gap) > 2 {
			printLine("  WARN plate does not rest on the table in the model")
		}
	} else {
		printf("  WARN no plate geometry: %v\n", err)
	}

	printLine("\n== start state")
	touching := collisions(geoms, moving, obstacles)
	if len(touching) == 0 {
		printLine("  ok: nothing on the arm or gripper touches an obstacle")
	}
	for _, c := range touching {
		if isFixedLink(c[0]) {
			printf("  ok: %s touches %s; the link only turns about the base axis\n", c[0], c[1])
			continue
		}
		printf("  FAIL %s touches %s; the planner will ignore this pair for the whole move\n", c[0], c[1])
		failures++
	}

	printf("\n== sweep: gripper pointing down at world (%.0f, %.0f)\n", *goalX, *goalY)
	var plans []namedPlan
	for _, h := range parseHeights(*heights) {
		z := tableTop + h
		goal := spatialmath.NewPose(r3.Vector{X: *goalX, Y: *goalY, Z: z}, &spatialmath.OrientationVectorDegrees{OZ: -1})
		traj, err := plan(ctx, r, goal)
		wantOK := h >= 0
		switch {
		case err == nil && wantOK:
			printf("  %+6.0f mm (z=%7.1f): planned, %d waypoints\n", h, z, len(traj))
			plans = append(plans, namedPlan{fmt.Sprintf("%+.0f mm", h), traj})
		case err == nil && !wantOK:
			printf("  %+6.0f mm (z=%7.1f): FAIL planned a goal inside the table\n", h, z)
			plans = append(plans, namedPlan{fmt.Sprintf("%+.0f mm", h), traj})
			failures++
		case err != nil && wantOK:
			printf("  %+6.0f mm (z=%7.1f): refused (gripper body reaches the table, or unreachable): %s\n", h, z, firstLine(err))
		default:
			printf("  %+6.0f mm (z=%7.1f): ok, refused: %s\n", h, z, firstLine(err))
		}
	}

	printLine("\n== paths: independent collision check of each planned trajectory")
	for _, p := range plans {
		hits, minClear, err := checkTrajectory(fs, p.traj, moving, obstacles, touching)
		if err != nil {
			return 0, err
		}
		if len(hits) == 0 {
			printf("  %-8s ok, closest approach to table %.1f mm\n", p.name, minClear)
			continue
		}
		failures++
		printf("  %-8s FAIL, closest approach to table %.1f mm\n", p.name, minClear)
		for _, h := range hits {
			printf("           %s\n", h)
		}
	}
	return failures, nil
}

type namedPlan struct {
	name string
	traj motionplan.Trajectory
}

func connect(ctx context.Context, logger logging.Logger) (*client.RobotClient, error) {
	if *credPath == "" {
		return client.New(ctx, *addr, logger, client.WithDialOptions(rpc.WithInsecure(), rpc.WithDisableDirectGRPC()))
	}
	f, err := os.Open(*credPath)
	if err != nil {
		return nil, err
	}
	defer func() { _ = f.Close() }() //nolint:errcheck
	var remote string
	var cred struct {
		Cloud struct {
			APIKey struct{ ID, Key string } `json:"api_key"`
		} `json:"cloud"`
	}
	sc := bufio.NewScanner(f)
	for prev := ""; sc.Scan(); prev = sc.Text() {
		line := strings.TrimSpace(sc.Text())
		if prev == "Remote address" {
			remote = line
		}
		if strings.HasPrefix(line, "{") {
			if err := json.Unmarshal([]byte(line), &cred); err != nil {
				return nil, err
			}
		}
	}
	if remote == "" || cred.Cloud.APIKey.ID == "" {
		return nil, errors.New("credentials file needs a Remote address line and a cloud api_key")
	}
	return client.New(ctx, remote, logger, client.WithDialOptions(rpc.WithEntityCredentials(
		cred.Cloud.APIKey.ID, rpc.Credentials{Type: rpc.CredentialsTypeAPIKey, Payload: cred.Cloud.APIKey.Key})))
}

// classifyFrames splits the frames that carry geometry into those carried by the arm and
// everything else.
func classifyFrames(fs *referenceframe.FrameSystem) (moving, obstacles []string) {
	for _, name := range fs.FrameNames() {
		if strings.HasSuffix(name, "_origin") {
			continue
		}
		frame := fs.Frame(name)
		chain, err := fs.TracebackFrame(frame)
		if err != nil {
			continue
		}
		onArm := slices.ContainsFunc(chain, func(f referenceframe.Frame) bool { return f.Name() == *armName })
		if onArm {
			moving = append(moving, name)
		} else {
			obstacles = append(obstacles, name)
		}
	}
	slices.Sort(moving)
	slices.Sort(obstacles)
	return moving, obstacles
}

func collisions(geoms map[string]*referenceframe.GeometriesInFrame, moving, obstacles []string) [][2]string {
	var out [][2]string
	for _, m := range moving {
		for _, o := range obstacles {
			for _, mg := range geomsOf(geoms, m) {
				for _, og := range geomsOf(geoms, o) {
					if hit, _, err := mg.CollidesWith(og, 0); err == nil && hit {
						out = append(out, [2]string{label(m, mg), label(o, og)})
					}
				}
			}
		}
	}
	return out
}

// checkTrajectory walks the trajectory with interpolation and reports arm/gripper hits on
// obstacles, skipping pairs that were already touching at the start (as the planner does).
func checkTrajectory(
	fs *referenceframe.FrameSystem, traj motionplan.Trajectory, moving, obstacles []string, ignore [][2]string,
) ([]string, float64, error) {
	seen := map[string]bool{}
	var hits []string
	minClear := math.Inf(1)
	for i := 0; i+1 < len(traj); i++ {
		from, to := traj[i].ToLinearInputs(), traj[i+1].ToLinearInputs()
		for s := 0; s <= *substeps; s++ {
			step, err := referenceframe.InterpolateFS(fs, from, to, float64(s)/float64(*substeps))
			if err != nil {
				return nil, 0, err
			}
			geoms, err := referenceframe.FrameSystemGeometriesLinearInputs(fs, step)
			if err != nil {
				return nil, 0, err
			}
			for _, tg := range geomsOf(geoms, *tableName) {
				for _, m := range moving {
					for _, mg := range geomsOf(geoms, m) {
						if isFixedLink(label(m, mg)) {
							continue
						}
						if d, err := mg.DistanceFrom(tg); err == nil {
							minClear = math.Min(minClear, d)
						}
					}
				}
			}
			for _, c := range collisions(geoms, moving, obstacles) {
				key := c[0] + " / " + c[1]
				if isFixedLink(c[0]) || slices.Contains(ignore, c) || seen[key] {
					continue
				}
				seen[key] = true
				hits = append(hits, fmt.Sprintf("waypoint %d→%d: %s hits %s", i, i+1, c[0], c[1]))
			}
		}
	}
	return hits, minClear, nil
}

func plan(ctx context.Context, r *client.RobotClient, goal spatialmath.Pose) (motionplan.Trajectory, error) {
	ms, err := motion.FromProvider(r, *motionName)
	if err != nil {
		return nil, err
	}
	req := motion.MoveReq{
		ComponentName: *gripperName,
		Destination:   referenceframe.NewPoseInFrame(referenceframe.World, goal),
	}
	pb, err := req.ToProto(*motionName)
	if err != nil {
		return nil, err
	}
	raw, err := protojson.Marshal(pb)
	if err != nil {
		return nil, err
	}
	resp, err := ms.DoCommand(ctx, map[string]any{"plan": string(raw)})
	if err != nil {
		return nil, err
	}
	var traj motionplan.Trajectory
	if err := mapstructure.Decode(resp["plan"], &traj); err != nil {
		return nil, fmt.Errorf("decoding plan: %w", err)
	}
	return traj, nil
}

func isFixedLink(geomLabel string) bool {
	for _, link := range strings.Split(*fixedLinks, ",") {
		if geomLabel == *armName+"/"+*armName+":"+strings.TrimSpace(link) {
			return true
		}
	}
	return false
}

// geomsOf returns a component's geometry. A component with a kinematic model carries it on
// its own frame; one with only a frame-config or Geometries() shape carries it on <name>_origin.
func geomsOf(geoms map[string]*referenceframe.GeometriesInFrame, component string) []spatialmath.Geometry {
	var out []spatialmath.Geometry
	for _, f := range []string{component, component + "_origin"} {
		if g, ok := geoms[f]; ok {
			out = append(out, g.Geometries()...)
		}
	}
	return out
}

func label(frame string, g spatialmath.Geometry) string {
	if g.Label() == "" || g.Label() == frame {
		return frame
	}
	return frame + "/" + g.Label()
}

// topOf and bottomOf assume the frame's geometry is an axis-aligned box.
func topOf(geoms map[string]*referenceframe.GeometriesInFrame, frame string) (float64, error) {
	_, hi, err := zExtent(geoms, frame)
	return hi, err
}

func bottomOf(geoms map[string]*referenceframe.GeometriesInFrame, frame string) (float64, error) {
	lo, _, err := zExtent(geoms, frame)
	return lo, err
}

func zExtent(geoms map[string]*referenceframe.GeometriesInFrame, frame string) (float64, float64, error) {
	gs := geomsOf(geoms, frame)
	if len(gs) == 0 {
		return 0, 0, fmt.Errorf("frame %q has no geometry", frame)
	}
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, g := range gs {
		box := g.ToProtobuf().GetBox()
		if box == nil {
			return 0, 0, fmt.Errorf("frame %q geometry %q is not a box", frame, g.Label())
		}
		z, half := g.Pose().Point().Z, box.GetDimsMm().GetZ()/2
		lo, hi = math.Min(lo, z-half), math.Max(hi, z+half)
	}
	return lo, hi, nil
}

func parseHeights(s string) []float64 {
	var out []float64
	for _, f := range strings.Split(s, ",") {
		var v float64
		if _, err := fmt.Sscan(strings.TrimSpace(f), &v); err == nil {
			out = append(out, v)
		}
	}
	return out
}

func fmtPose(p spatialmath.Pose) string {
	pt, ov := p.Point(), p.Orientation().OrientationVectorDegrees()
	return fmt.Sprintf("(%7.1f, %7.1f, %7.1f)  ov (%.2f, %.2f, %.2f) θ %.1f", pt.X, pt.Y, pt.Z, ov.OX, ov.OY, ov.OZ, ov.Theta)
}

func firstLine(err error) string {
	s := err.Error()
	if i := strings.IndexByte(s, '\n'); i >= 0 {
		s = s[:i]
	}
	if len(s) > 110 {
		s = s[:110] + "…"
	}
	return s
}

func printf(format string, args ...any) {
	fmt.Fprintf(os.Stdout, format, args...) //nolint:errcheck
}

func printLine(args ...any) {
	fmt.Fprintln(os.Stdout, args...) //nolint:errcheck
}
