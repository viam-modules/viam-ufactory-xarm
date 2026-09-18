package arm

import (
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
)

// attachVisualsAndProperties puts the rest of what this arm knows about itself onto the model, so
// it reaches viewers and the motion service through GetKinematics instead of a separate RPC and
// module config. The shipped GLB parts become visual geometry on their links, and the move rate
// becomes the trajectory sampling frequency. Joint limits are set in MakeModelFrame.
func attachVisualsAndProperties(model referenceframe.Model, conf *Config, modelName string, logger logging.Logger) error {
	sm, ok := model.(*referenceframe.SimpleModel)
	if !ok {
		return fmt.Errorf("cannot attach kinematics to a %T", model)
	}

	// the GLB files ship inside the module, so a missing one means we are running outside a
	// module root, as in tests, and the model simply has no visuals
	for _, part := range armTo3DModelParts[modelName] {
		// a model loaded from an SVA v2 file already carries its visuals by path
		if len(sm.VisualGeometries(part)) > 0 {
			continue
		}
		mesh, err := threeDMeshFromName(modelName, part)
		if err != nil {
			logger.Debugw("no visual mesh for link", "link", part, "error", err)
			continue
		}
		geometry := &commonpb.Geometry{GeometryType: &commonpb.Geometry_Mesh{Mesh: &commonpb.Mesh{
			ContentType: "glb",
			Mesh:        mesh.Mesh,
			SourcePath:  fmt.Sprintf("3d_models/%s/%s.glb", modelName, part),
		}}}
		if err := sm.SetVisualGeometries(part, []*commonpb.Geometry{geometry}); err != nil {
			logger.Warnw("visual mesh names a link the model does not have", "link", part, "error", err)
		}
	}

	hz := conf.moveHZ()
	sm.SetKinematicProperties(&commonpb.KinematicProperties{TrajectorySamplingFreqHz: &hz})
	return nil
}
