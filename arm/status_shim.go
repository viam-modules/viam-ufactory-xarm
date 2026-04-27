package arm

import "context"

// Status methods added so the module compiles against rdk versions that include
// Status in the Resource interface. Empty map is a safe default for components
// that don't have a meaningful status to report.

func (x *xArm) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}

func (g *myGripperLite) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}

func (g *myGripper) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}

func (g *myVacuumGripper) Status(_ context.Context) (map[string]any, error) {
	return map[string]any{}, nil
}
