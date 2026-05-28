package arm

import (
	"context"
	"encoding/binary"
	"fmt"
	"regexp"
	"strconv"
	"strings"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
)

type ArmModel string

const (
	ArmModelUnknown ArmModel = "unknown"
	ArmModelXArm5   ArmModel = "xArm5"
	ArmModelXArm6   ArmModel = "xArm6"
	ArmModelXArm7   ArmModel = "xArm7"
	ArmModelXArm7T  ArmModel = "xArm7T"
	ArmModelLite6   ArmModel = "lite6"
	ArmModelXArm850 ArmModel = "xArm850"
)

type GripperKind string

const (
	GripperKindUnknown  GripperKind = "unknown"
	GripperKindStandard GripperKind = "standard"
	GripperKindBio      GripperKind = "bio"
	GripperKindVacuum   GripperKind = "vacuum"
)

type DetectedArm struct {
	Model           ArmModel
	DeviceType      byte
	Axis            byte
	Submodel        string
	ArmTypeCode     int
	ControlTypeCode int
	FirmwareVersion string
}

type DetectedGripper struct {
	Kind     GripperKind
	Version  string
	Submodel string
}

func decodeArmModel(deviceType, axis byte) ArmModel {
	switch deviceType {
	case 3:
		return ArmModelXArm7
	case 5:
		return ArmModelXArm5
	case 6:
		return ArmModelXArm6
	case 13:
		return ArmModelXArm7T
	case 9:
		if axis == 6 {
			return ArmModelLite6
		}
	case 12:
		if axis == 6 {
			return ArmModelXArm850
		}
	}
	return ArmModelUnknown
}

// armModelFromSNPrefix is the authoritative model signal. GET_HD_TYPES is
// harmonic-drive debug data (xarm-python-sdk doc/api/xarm_api.md:1757) and
// the banner's leading "axis" digit is "1" on firmware 2.x; the SN prefix
// from xarm.py:1841-1844 is what the SDKs actually trust.
func armModelFromSNPrefix(armTypeStr string) (ArmModel, byte) {
	if len(armTypeStr) < 2 {
		return ArmModelUnknown, 0
	}
	switch armTypeStr[:2] {
	case "XF":
		return ArmModelXArm5, 5
	case "XI":
		return ArmModelXArm6, 6
	case "XS":
		return ArmModelXArm7, 7
	case "CS":
		return ArmModelXArm7T, 7
	case "LI":
		return ArmModelLite6, 6
	case "FX":
		return ArmModelXArm850, 6
	}
	return ArmModelUnknown, 0
}

func (x *xArm) detectArm(ctx context.Context) (DetectedArm, error) {
	v, err := x.detectVersion(ctx)
	if err != nil {
		return DetectedArm{Model: ArmModelUnknown}, err
	}
	d := DetectedArm{
		DeviceType:      byte(v.deviceType),
		Submodel:        v.armTypeStr,
		ArmTypeCode:     v.armTypeCode,
		ControlTypeCode: v.controlTypeCode,
		FirmwareVersion: v.firmwareVersion,
	}
	d.Model, d.Axis = armModelFromSNPrefix(v.armTypeStr)
	return d, nil
}

type versionInfo struct {
	axis            int
	deviceType      int
	armTypeStr      string
	controlTypeStr  string
	armTypeCode     int
	controlTypeCode int
	firmwareVersion string
}

// [^,]+ instead of the C++ SDK's greedy .* — RE2 doesn't backtrack the same way.
var versionBannerRE = regexp.MustCompile(`(\d+),(\d+),([^,]+),([^,]+),.*?[vV]?(\d+)\.(\d+)\.(\d+)`)

func parseVersionBanner(banner string) (versionInfo, bool) {
	m := versionBannerRE.FindStringSubmatch(banner)
	if m == nil {
		return versionInfo{}, false
	}
	v := versionInfo{
		armTypeStr:      m[3],
		controlTypeStr:  m[4],
		firmwareVersion: fmt.Sprintf("%s.%s.%s", m[5], m[6], m[7]),
	}
	v.axis, _ = strconv.Atoi(m[1])
	v.deviceType, _ = strconv.Atoi(m[2])
	if len(v.armTypeStr) >= 6 {
		v.armTypeCode, _ = strconv.Atoi(v.armTypeStr[2:6])
	}
	if len(v.controlTypeStr) >= 6 {
		v.controlTypeCode, _ = strconv.Atoi(v.controlTypeStr[2:6])
	}
	return v, true
}

func (x *xArm) detectVersion(ctx context.Context) (versionInfo, error) {
	c := x.newCmd(regMap["Version"])
	resp, err := x.send(ctx, c, true)
	if err != nil {
		return versionInfo{}, err
	}
	if len(resp.params) < 2 {
		return versionInfo{}, fmt.Errorf("Version response too short: %d (%v)", len(resp.params), resp.params)
	}
	banner := strings.TrimRight(string(resp.params[1:]), "\x00 ")
	v, ok := parseVersionBanner(banner)
	if !ok {
		return versionInfo{}, fmt.Errorf("could not parse version banner: %q", banner)
	}
	return v, nil
}

func (x *xArm) detectStandardGripper(ctx context.Context) (DetectedGripper, error) {
	const numRegs = 3
	c := x.gripperPreamble(false)
	c.params = append(c.params, 0x08, 0x01)
	c.params = append(c.params, 0x00, numRegs)
	res, err := x.gripperSend(ctx, c)
	if err != nil {
		return DetectedGripper{Kind: GripperKindUnknown}, err
	}
	const headerLen = 5
	wantLen := headerLen + 2*numRegs
	if len(res.params) < wantLen {
		return DetectedGripper{Kind: GripperKindUnknown},
			fmt.Errorf("standard gripper version response too short: got %d, want %d (%v)", len(res.params), wantLen, res.params)
	}
	data := res.params[headerLen : headerLen+2*numRegs]
	major := binary.BigEndian.Uint16(data[0:2])
	minor := binary.BigEndian.Uint16(data[2:4])
	patch := binary.BigEndian.Uint16(data[4:6])
	return DetectedGripper{
		Kind:     GripperKindStandard,
		Version:  fmt.Sprintf("%d.%d.%d", major, minor, patch),
		Submodel: standardGripperSubmodel(major, minor, patch),
	}, nil
}

// standardGripperSubmodel splits at firmware >= 3.4.3, the gate the Python
// SDK uses for status-reporting support (gripper.py:83-85).
func standardGripperSubmodel(major, minor, patch uint16) string {
	switch {
	case major > 3:
		return "v2"
	case major == 3 && minor > 4:
		return "v2"
	case major == 3 && minor == 4 && patch >= 3:
		return "v2"
	default:
		return "v1"
	}
}

// detectBioGripper: byteCount==2 means v1 (single-register response), 2*numRegs
// means v2 (full SN). Matches xarm_bio.cc:_get_bio_gripper_sn.
func (x *xArm) detectBioGripper(ctx context.Context) (DetectedGripper, error) {
	const numRegs = 16
	c := x.gripperPreamble(false)
	c.params = append(c.params, 0x0B, 0x10)
	c.params = append(c.params, 0x00, numRegs)
	res, err := x.gripperSend(ctx, c)
	if err != nil {
		return DetectedGripper{Kind: GripperKindUnknown}, err
	}
	const headerLen = 5
	if len(res.params) < headerLen {
		return DetectedGripper{Kind: GripperKindUnknown},
			fmt.Errorf("bio gripper response too short: %d (%v)", len(res.params), res.params)
	}
	byteCount := res.params[headerLen-1]
	switch byteCount {
	case 2:
		return DetectedGripper{Kind: GripperKindBio, Version: "1"}, nil
	case 2 * numRegs:
		if len(res.params) < headerLen+int(byteCount) {
			return DetectedGripper{Kind: GripperKindUnknown},
				fmt.Errorf("bio gripper response truncated: got %d, want %d", len(res.params), headerLen+int(byteCount))
		}
		sn := strings.TrimRight(string(res.params[headerLen:headerLen+int(byteCount)]), "\x00 ")
		return DetectedGripper{Kind: GripperKindBio, Version: "2 sn=" + sn}, nil
	default:
		return DetectedGripper{Kind: GripperKindUnknown},
			fmt.Errorf("bio gripper unexpected byte count: %d (%v)", byteCount, res.params)
	}
}

func probeGripper(ctx context.Context, a arm.Arm, kind GripperKind, logger logging.Logger) DetectedGripper {
	x, ok := a.(*xArm)
	if !ok {
		logger.Warnf("%s gripper detection skipped: arm dependency is not a *xArm (got %T)", kind, a)
		return DetectedGripper{Kind: GripperKindUnknown}
	}
	var (
		d   DetectedGripper
		err error
	)
	switch kind {
	case GripperKindStandard:
		d, err = x.detectStandardGripper(ctx)
	case GripperKindBio:
		d, err = x.detectBioGripper(ctx)
	case GripperKindVacuum:
		d, err = x.detectVacuumGripper(ctx)
	default:
		logger.Warnf("gripper detection skipped: unknown kind %q", kind)
		return DetectedGripper{Kind: GripperKindUnknown}
	}
	if err != nil {
		logger.Warnf("%s gripper detection failed: %v", kind, err)
		return d
	}
	logger.Infof("%s gripper detected (submodel=%q version=%q)", d.Kind, d.Submodel, d.Version)
	return d
}

func (x *xArm) detectVacuumGripper(ctx context.Context) (DetectedGripper, error) {
	c := x.newCmd(regMap["VacuumState"])
	c.params = append(c.params, 0x09, 0x0A, 0x18)
	resp, err := x.send(ctx, c, true)
	if err != nil {
		return DetectedGripper{Kind: GripperKindUnknown}, err
	}
	if len(resp.params) < 5 {
		return DetectedGripper{Kind: GripperKindUnknown},
			fmt.Errorf("vacuum gripper response too short: %d (%v)", len(resp.params), resp.params)
	}
	return DetectedGripper{
		Kind:     GripperKindVacuum,
		Submodel: vacuumGripperSubmodel(x.detectedArm),
	}, nil
}

// vacuumGripperSubmodel infers hardware revision from the arm: xarm-python-sdk
// gpio.py:117 uses TGPIO outputs 3/4 (v2) for 850 and for xArm6/7 with
// submodel >= 1305; older xArms use TGPIO 0/1 (v1); Lite 6 has its own bus.
func vacuumGripperSubmodel(arm DetectedArm) string {
	switch {
	case arm.Model == ArmModelLite6:
		return "lite"
	case arm.Model == ArmModelXArm850:
		return "v2"
	case arm.ArmTypeCode >= 1305:
		return "v2"
	default:
		return "v1"
	}
}
