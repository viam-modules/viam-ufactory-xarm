package arm

import (
	"testing"

	"go.viam.com/test"
)

func TestParseGripperUint16Response(t *testing.T) {
	t.Run("valid response", func(t *testing.T) {
		// Last two bytes are the register value.
		v, err := parseGripperUint16Response([]byte{0x00, 0x09, 0x08, 0x03, 0x02, 0x07, 0xD0}, "getGripperSpeed")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, v, test.ShouldEqual, 2000)
	})

	t.Run("invalid response length", func(t *testing.T) {
		_, err := parseGripperUint16Response([]byte{0x00, 0x09}, "getGripperForce")
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "unexpected length for getGripperForce response")
	})
}
