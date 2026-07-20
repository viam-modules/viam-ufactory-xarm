package arm

import (
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

func TestResolveGripperConnectionType(t *testing.T) {
	logger := logging.NewTestLogger(t)

	// Explicit override wins over detection.
	test.That(t, resolveGripperConnectionType("contact", submodelV1, VacuumGripperModel, logger),
		test.ShouldEqual, connectionContact)
	test.That(t, resolveGripperConnectionType("plugin", submodelV2, VacuumGripperModel, logger),
		test.ShouldEqual, connectionPlugin)

	// Auto-detect from submodel.
	test.That(t, resolveGripperConnectionType("", submodelV2, VacuumGripperModel, logger),
		test.ShouldEqual, connectionContact)
	test.That(t, resolveGripperConnectionType("", submodelV1, VacuumGripperModel, logger),
		test.ShouldEqual, connectionPlugin)

	// Lite always uses plug-in, even if a connection_type is set.
	test.That(t, resolveGripperConnectionType("contact", submodelLite, VacuumGripperModelLite, logger),
		test.ShouldEqual, connectionPlugin)

	// Lite auto-detect (empty config) also stays plug-in.
	test.That(t, resolveGripperConnectionType("", submodelLite, VacuumGripperModelLite, logger),
		test.ShouldEqual, connectionPlugin)
}
