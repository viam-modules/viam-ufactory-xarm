package arm

import (
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

func TestParseVersionBanner(t *testing.T) {
	tests := []struct {
		name        string
		banner      string
		wantOK      bool
		wantAxis    int
		wantDevType int
		wantArmStr  string
		wantArmCode int
		wantCtlCode int
		wantFW      string
	}{
		{
			name:        "xArm6 model 1 (XI1300)",
			banner:      "1,6,XI1300_2020Sx0001,CI1300_2020Sx0001,v1.10.0",
			wantOK:      true,
			wantAxis:    1,
			wantDevType: 6,
			wantArmStr:  "XI1300_2020Sx0001",
			wantArmCode: 1300,
			wantCtlCode: 1300,
			wantFW:      "1.10.0",
		},
		{
			name:        "xArm6 model 3 (XI1304)",
			banner:      "1,6,XI1304_2021Sx0001,CI1300_2021Sx0001,v1.13.20",
			wantOK:      true,
			wantAxis:    1,
			wantDevType: 6,
			wantArmStr:  "XI1304_2021Sx0001",
			wantArmCode: 1304,
			wantCtlCode: 1300,
			wantFW:      "1.13.20",
		},
		{
			name:        "lite6",
			banner:      "1,9,LI1100_2021Sx0001,CI1300_2021Sx0001,v1.13.20",
			wantOK:      true,
			wantAxis:    1,
			wantDevType: 9,
			wantArmStr:  "LI1100_2021Sx0001",
			wantArmCode: 1100,
			wantCtlCode: 1300,
			wantFW:      "1.13.20",
		},
		{
			name:        "no leading v",
			banner:      "1,6,XI1305_2022Sx0001,CI1300_2022Sx0001,1.14.0",
			wantOK:      true,
			wantArmCode: 1305,
			wantCtlCode: 1300,
			wantFW:      "1.14.0",
		},
		{name: "old protocol firmware (no banner)", banner: "v1.5.0", wantOK: false},
		{name: "empty", banner: "", wantOK: false},
		{name: "garbage", banner: "hello world", wantOK: false},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got, err := parseVersionBanner(tc.banner, logging.NewTestLogger(t))
			if !tc.wantOK {
				test.That(t, err, test.ShouldNotBeNil)
				return
			}
			test.That(t, err, test.ShouldBeNil)
			if tc.wantAxis != 0 {
				test.That(t, got.axis, test.ShouldEqual, tc.wantAxis)
			}
			if tc.wantDevType != 0 {
				test.That(t, got.deviceType, test.ShouldEqual, tc.wantDevType)
			}
			if tc.wantArmStr != "" {
				test.That(t, got.armTypeStr, test.ShouldEqual, tc.wantArmStr)
			}
			test.That(t, got.armTypeCode, test.ShouldEqual, tc.wantArmCode)
			test.That(t, got.controlTypeCode, test.ShouldEqual, tc.wantCtlCode)
			test.That(t, got.firmwareVersion, test.ShouldEqual, tc.wantFW)
		})
	}
}

func TestStandardGripperSubmodel(t *testing.T) {
	tests := []struct {
		name              string
		major, minor, pat uint16
		want              string
	}{
		{"old major", 2, 9, 9, "v1"},
		{"3.4.2 just below gate", 3, 4, 2, "v1"},
		{"3.4.3 exact gate", 3, 4, 3, "v2"},
		{"3.5.0 above gate", 3, 5, 0, "v2"},
		{"4.0.0", 4, 0, 0, "v2"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			test.That(t, standardGripperSubmodel(tc.major, tc.minor, tc.pat), test.ShouldEqual, tc.want)
		})
	}
}

func TestVacuumGripperSubmodel(t *testing.T) {
	tests := []struct {
		name string
		arm  detectedArm
		want string
	}{
		{"lite6", detectedArm{model: hardwareModelLite6}, "lite"},
		{"850", detectedArm{model: hardwareModelXArm850}, "v2"},
		{"xArm6 XI1305", detectedArm{model: hardwareModelXArm6, armTypeCode: 1305}, "v2"},
		{"xArm6 XI1304", detectedArm{model: hardwareModelXArm6, armTypeCode: 1304}, "v1"},
		{"xArm7 no submodel info", detectedArm{model: hardwareModelXArm7}, "v1"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			test.That(t, vacuumGripperSubmodel(tc.arm), test.ShouldEqual, tc.want)
		})
	}
}

func TestHardwareModelFromSNPrefix(t *testing.T) {
	tests := []struct {
		name       string
		armTypeStr string
		wantModel  hardwareModel
		wantAxis   byte
	}{
		{"xArm5 (XF)", "XF1300_2021Sx0001", hardwareModelXArm5, 5},
		{"xArm6 (XI)", "XI1305_2021Sx0001", hardwareModelXArm6, 6},
		{"xArm7 (XS)", "XS1300_2021Sx0001", hardwareModelXArm7, 7},
		{"xArm7T (CS)", "CS1300_2021Sx0001", hardwareModelXArm7T, 7},
		{"lite6 (LI)", "LI1100_2021Sx0001", hardwareModelLite6, 6},
		{"xArm850 (FX)", "FX1200_2021Sx0001", hardwareModelXArm850, 6},
		{"unknown prefix", "ZZ1300_2021Sx0001", hardwareModelUnknown, 0},
		{"empty", "", hardwareModelUnknown, 0},
		{"too short", "X", hardwareModelUnknown, 0},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			gotModel, gotAxis := armModelFromSNPrefix(tc.armTypeStr)
			test.That(t, gotModel, test.ShouldEqual, tc.wantModel)
			test.That(t, gotAxis, test.ShouldEqual, tc.wantAxis)
		})
	}
}

func TestDecodehardwareModel(t *testing.T) {
	tests := []struct {
		name       string
		deviceType byte
		axis       byte
		want       hardwareModel
	}{
		{"xArm5", 5, 5, hardwareModelXArm5},
		{"xArm6", 6, 6, hardwareModelXArm6},
		{"xArm7", 3, 7, hardwareModelXArm7},
		{"xArm7T", 13, 7, hardwareModelXArm7T},
		{"lite6", 9, 6, hardwareModelLite6},
		{"xArm850", 12, 6, hardwareModelXArm850},
		{"lite6 wrong axis falls back", 9, 7, hardwareModelUnknown},
		{"xArm850 wrong axis falls back", 12, 7, hardwareModelUnknown},
		{"unknown device_type", 99, 6, hardwareModelUnknown},
		{"zero values", 0, 0, hardwareModelUnknown},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got := decodeHardwareModel(tc.deviceType, tc.axis)
			test.That(t, got, test.ShouldEqual, tc.want)
		})
	}
}
