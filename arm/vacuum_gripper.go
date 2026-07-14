package arm

import (
	"context"
	"errors"
	"fmt"
	"sync/atomic"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// VacuumGripperModel model for the ufactory vacuum gripper.
var VacuumGripperModel = family.WithModel("vacuum_gripper")

// VacuumGripperModelLite is the ufactory vacuum gripper commonly attached to the lite6.
var VacuumGripperModelLite = family.WithModel("vacuum_gripper_lite")

var caseBoxSize = r3.Vector{X: 70, Y: 93, Z: 117}
var liteCaseBoxSize = r3.Vector{X: 51, Y: 51, Z: 54}

// connectionType selects which TGPIO pin-set drives the vacuum gripper.
// Plug-in (v1) uses user pins 0/1; contact (v2) uses user pins 3/4.
type connectionType string

const (
	connectionPlugin  connectionType = "plugin"
	connectionContact connectionType = "contact"
)

// vacuumPins returns the [ON-pin, OFF-pin] TGPIO user-pin pair for a connection type.
func vacuumPins(ct connectionType) [2]int {
	if ct == connectionContact {
		return [2]int{3, 4}
	}
	return [2]int{0, 1}
}

// resolveGripperConnectionType picks the effective connection type from the
// configured override, falling back to the detected submodel. The Lite always
// uses the plug-in path.
func resolveGripperConnectionType(configured, detectedSubmodel string, model resource.Model, logger logging.Logger) connectionType {
	if model == VacuumGripperModelLite {
		if configured != "" {
			logger.Warnf("connection_type %q ignored for %s; Lite uses the plug-in path", configured, model.Name)
		}
		return connectionPlugin
	}
	switch configured {
	case string(connectionContact):
		return connectionContact
	case string(connectionPlugin):
		return connectionPlugin
	default:
		if detectedSubmodel == submodelV2 {
			logger.Info("vacuum gripper: auto-detected contact (v2) connection; set connection_type in config to override")
			return connectionContact
		}
		return connectionPlugin
	}
}

func init() {
	resource.RegisterComponent(
		gripper.API,
		VacuumGripperModel,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newVacuumGripper,
		})
	resource.RegisterComponent(
		gripper.API,
		VacuumGripperModelLite,
		resource.Registration[gripper.Gripper, *GripperConfig]{
			Constructor: newVacuumGripper,
		})
}

func newVacuumGripper(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (gripper.Gripper, error) {
	newConf, err := resource.NativeConfig[*GripperConfig](config)
	if err != nil {
		return nil, err
	}

	g := &myVacuumGripper{
		name:   config.ResourceName(),
		conf:   newConf,
		logger: logger,
		model:  config.Model,
	}

	g.arm, err = arm.FromProvider(deps, newConf.Arm)
	if err != nil {
		return nil, err
	}

	g.detected = probeGripper(ctx, g.arm, gripperKindVacuum, logger)

	g.connType = resolveGripperConnectionType(newConf.ConnectionType, g.detected.submodel, config.Model, logger)

	return g, nil
}

type myVacuumGripper struct {
	resource.AlwaysRebuild

	name  resource.Name
	conf  *GripperConfig
	model resource.Model

	arm arm.Arm

	isMoving atomic.Bool

	detected detectedGripper
	connType connectionType

	logger logging.Logger
}

func (g *myVacuumGripper) Grab(ctx context.Context, extra map[string]any) (bool, error) {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	{
		_, err := g.arm.DoCommand(ctx, map[string]any{
			grabVacuumKey:     true,
			connectionTypeKey: string(g.connType),
		})
		if err != nil {
			return false, err
		}
	}

	return true, nil
}

func (g *myVacuumGripper) Open(ctx context.Context, extra map[string]any) error {
	g.isMoving.Store(true)
	defer g.isMoving.Store(false)

	{
		_, err := g.arm.DoCommand(ctx, map[string]any{
			openVacuumKey:     true,
			connectionTypeKey: string(g.connType),
		})
		if err != nil {
			return err
		}
	}
	return nil
}

func (g *myVacuumGripper) IsHoldingSomething(
	ctx context.Context,
	extra map[string]any,
) (gripper.HoldingStatus, error) {
	res, err := g.arm.DoCommand(ctx, map[string]any{
		getVacuumGripperStateKey: true,
		connectionTypeKey:        string(g.connType),
	})
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	return gripper.HoldingStatus{IsHoldingSomething: res[vacuumGripperStateKey].(bool)}, nil
}

func (g *myVacuumGripper) Name() resource.Name {
	return g.name
}

func (g *myVacuumGripper) Close(ctx context.Context) error {
	return g.Stop(ctx, nil)
}

func (g *myVacuumGripper) DoCommand(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	return map[string]any{}, nil
}

func (g *myVacuumGripper) IsMoving(context.Context) (bool, error) {
	return g.isMoving.Load(), nil
}

func (g *myVacuumGripper) Stop(context.Context, map[string]any) error {
	// TODO(erh): fix me
	return nil
}

func (g *myVacuumGripper) Geometries(ctx context.Context, _ map[string]any) ([]spatialmath.Geometry, error) {
	var (
		caseBox spatialmath.Geometry
		err     error
	)
	switch g.model {
	case VacuumGripperModel:
		caseBox, err = spatialmath.NewBox(spatialmath.NewPoseFromPoint(
			r3.Vector{X: 0, Y: 0, Z: -1 * (g.conf.VacuumLengthMM + caseBoxSize.Z/2)}),
			caseBoxSize,
			"vacuum-gripper-box")
		if err != nil {
			return nil, err
		}
	case VacuumGripperModelLite:
		caseBox, err = spatialmath.NewBox(spatialmath.NewPoseFromPoint(
			r3.Vector{X: 0, Y: 0, Z: -1 * (g.conf.VacuumLengthMM + liteCaseBoxSize.Z/2)}),
			liteCaseBoxSize,
			"vacuum-gripper-box")
		if err != nil {
			return nil, err
		}
	default:
		return nil, fmt.Errorf("unsupported model %s", g.model)
	}

	vacuum, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(
		r3.Vector{X: 0, Y: 0, Z: -1 * (g.conf.VacuumLengthMM / 2)}),
		r3.Vector{X: 5, Y: 5, Z: max(5, g.conf.VacuumLengthMM)},
		"vacuum-gripper-tube")
	if err != nil {
		return nil, err
	}

	return []spatialmath.Geometry{
		caseBox, vacuum,
	}, nil
}

func (g *myVacuumGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, errors.ErrUnsupported
}

func (g *myVacuumGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return nil, errors.ErrUnsupported
}

func (g *myVacuumGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return errors.ErrUnsupported
}

func (g *myVacuumGripper) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}
