package arm

import (
	"testing"

	"go.viam.com/test"
)

func TestGripperConfigValidateConnectionType(t *testing.T) {
	base := func(ct string) *GripperConfig {
		return &GripperConfig{Arm: "a", ConnectionType: ct}
	}
	for _, ok := range []string{"", "plugin", "contact"} {
		_, _, err := base(ok).Validate("p")
		test.That(t, err, test.ShouldBeNil)
	}
	_, _, err := base("wired").Validate("p")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "connection_type")
}

func TestGripperConfigValidateVersionAndForce(t *testing.T) {
	_, _, err := (&GripperConfig{Arm: "a", GripperVersion: "G3"}).Validate("p")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_version")

	_, _, err = (&GripperConfig{Arm: "a", GripperForce: 101}).Validate("p")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper_force")
}
