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
	"go.viam.com/rdk/utils"
)

// hardwareModel identifies an xArm hardware family.
type hardwareModel string

// Known hardwareModel values.
const (
	hardwareModelUnknown hardwareModel = "unknown"
	hardwareModelXArm5   hardwareModel = "xArm5"
	hardwareModelXArm6   hardwareModel = "xArm6"
	hardwareModelXArm7   hardwareModel = "xArm7"
	hardwareModelXArm7T  hardwareModel = "xArm7T"
	hardwareModelLite6   hardwareModel = "lite6"
	hardwareModelXArm850 hardwareModel = "xArm850"
)

// gripperKind identifies a gripper hardware family.
type gripperKind string

// Known gripperKind values.
const (
	gripperKindUnknown  gripperKind = "unknown"
	gripperKindStandard gripperKind = "standard"
	gripperKindBio      gripperKind = "bio"
	gripperKindVacuum   gripperKind = "vacuum"
)

// Submodel labels reported in detectedGripper.Submodel.
const (
	submodelV1   = "v1"
	submodelV2   = "v2"
	submodelLite = "lite"
)

// Two-letter SN prefixes UFactory burns into the controller banner. These are
// the authoritative model signal — numeric device_type drifts across firmware
// versions, but the prefix does not.
const (
	snPrefixXArm5   = "XF"
	snPrefixXArm6   = "XI"
	snPrefixXArm7   = "XS"
	snPrefixXArm7T  = "CS"
	snPrefixLite6   = "LI"
	snPrefixXArm850 = "FX"
)

// detectedArm is the result of an arm-model probe.
type detectedArm struct {
	Model           hardwareModel
	DeviceType      byte
	Axis            byte
	Submodel        string
	ArmTypeCode     int
	ControlTypeCode int
	FirmwareVersion string
}

// detectedGripper is the result of a gripper-hardware probe.
type detectedGripper struct {
	Kind     gripperKind
	Version  string
	Submodel string
}

// unknownGripper returns a detectedGripper whose Kind is gripperKindUnknown. Use
// this instead of detectedGripper{} so the default carries the explicit label.
func unknownGripper() detectedGripper {
	return detectedGripper{Kind: gripperKindUnknown}
}

func decodeHardwareModel(deviceType, axis byte) hardwareModel {
	switch deviceType {
	case 3:
		return hardwareModelXArm7
	case 5:
		return hardwareModelXArm5
	case 6:
		return hardwareModelXArm6
	case 13:
		return hardwareModelXArm7T
	case 9:
		if axis == 6 {
			return hardwareModelLite6
		}
	case 12:
		if axis == 6 {
			return hardwareModelXArm850
		}
	}
	return hardwareModelUnknown
}

// armModelFromSNPrefix is the authoritative model signal. GET_HD_TYPES is
// harmonic-drive debug data, and the banner's leading "axis" digit is "1" on
// firmware 2.x; the two-letter SN prefix is what UFactory's SDKs actually trust.
func armModelFromSNPrefix(armTypeStr string) (hardwareModel, byte) {
	if len(armTypeStr) < 2 {
		return hardwareModelUnknown, 0
	}
	switch armTypeStr[:2] {
	case snPrefixXArm5:
		return hardwareModelXArm5, 5
	case snPrefixXArm6:
		return hardwareModelXArm6, 6
	case snPrefixXArm7:
		return hardwareModelXArm7, 7
	case snPrefixXArm7T:
		return hardwareModelXArm7T, 7
	case snPrefixLite6:
		return hardwareModelLite6, 6
	case snPrefixXArm850:
		return hardwareModelXArm850, 6
	}
	return hardwareModelUnknown, 0
}

func (x *xArm) detectArm(ctx context.Context) (detectedArm, error) {
	v, err := x.detectVersion(ctx)
	if err != nil {
		return detectedArm{Model: hardwareModelUnknown}, err
	}
	d := detectedArm{
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

// regex to parse xArm serial number format
var versionBannerRE = regexp.MustCompile(`(\d+),(\d+),([^,]+),([^,]+),.*?[vV]?(\d+)\.(\d+)\.(\d+)`)

func parseVersionBanner(banner string, logger logging.Logger) (versionInfo, error) {
	m := versionBannerRE.FindStringSubmatch(banner)
	if m == nil {
		return versionInfo{}, fmt.Errorf("could not parse version banner: %q", banner)
	}
	v := versionInfo{
		armTypeStr:      m[3],
		controlTypeStr:  m[4],
		firmwareVersion: fmt.Sprintf("%s.%s.%s", m[5], m[6], m[7]),
	}
	var err error
	if v.axis, err = strconv.Atoi(m[1]); err != nil {
		return versionInfo{}, fmt.Errorf("invalid axis %q in banner %q: %w", m[1], banner, err)
	}
	if v.deviceType, err = strconv.Atoi(m[2]); err != nil {
		return versionInfo{}, fmt.Errorf("invalid device_type %q in banner %q: %w", m[2], banner, err)
	}
	v.armTypeCode = parseSubmodelCode(v.armTypeStr, "arm", logger)
	v.controlTypeCode = parseSubmodelCode(v.controlTypeStr, "control", logger)
	return v, nil
}

// parseSubmodelCode extracts the 4-digit numeric code from SN positions [2:6]. A
// non-digit value means UFactory shipped a SN format we don't recognize — log and
// fall back to 0 so detection can still proceed using the 2-char prefix.
func parseSubmodelCode(s, kind string, logger logging.Logger) int {
	if len(s) < 6 {
		return 0
	}
	code, err := strconv.Atoi(s[2:6])
	if err != nil {
		logger.Warnf("could not parse %s submodel code from SN %q: %v", kind, s, err)
		return 0
	}
	return code
}

func (x *xArm) detectVersion(ctx context.Context) (versionInfo, error) {
	c := x.newCmd(regMap["Version"])
	resp, err := x.send(ctx, c, true)
	if err != nil {
		return versionInfo{}, err
	}
	if len(resp.params) < 2 {
		return versionInfo{}, fmt.Errorf("version response too short: %d (%v)", len(resp.params), resp.params)
	}
	banner := strings.TrimRight(string(resp.params[1:]), "\x00 ")
	return parseVersionBanner(banner, x.logger)
}

// Modbus register start addresses used by gripper probes.
const (
	standardGripperVersionReg uint16 = 0x0801 // SOFT_VER, 3 consecutive regs: major.minor.patch
	bioGripperSNReg           uint16 = 0x0B10 // BIO serial-number block, up to 16 regs
)

func (x *xArm) detectStandardGripper(ctx context.Context) (detectedGripper, error) {
	const numRegs = 3
	c := x.gripperPreamble(false)
	c.params = binary.BigEndian.AppendUint16(c.params, standardGripperVersionReg)
	c.params = binary.BigEndian.AppendUint16(c.params, numRegs)
	res, err := x.gripperSend(ctx, c)
	if err != nil {
		return unknownGripper(), err
	}
	const headerLen = 5
	wantLen := headerLen + 2*numRegs
	if len(res.params) < wantLen {
		return unknownGripper(),
			fmt.Errorf("standard gripper version response too short: got %d, want %d (%v)", len(res.params), wantLen, res.params)
	}
	data := res.params[headerLen : headerLen+2*numRegs]
	major := binary.BigEndian.Uint16(data[0:2])
	minor := binary.BigEndian.Uint16(data[2:4])
	patch := binary.BigEndian.Uint16(data[4:6])
	return detectedGripper{
		Kind:     gripperKindStandard,
		Version:  fmt.Sprintf("%d.%d.%d", major, minor, patch),
		Submodel: standardGripperSubmodel(major, minor, patch),
	}, nil
}

// standardGripperSubmodel splits at firmware >= 3.4.3, the gate for
// status-register (0x0000) support on the standard gripper.
func standardGripperSubmodel(major, minor, patch uint16) string {
	switch {
	case major > 3:
		return submodelV2
	case major == 3 && minor > 4:
		return submodelV2
	case major == 3 && minor == 4 && patch >= 3:
		return submodelV2
	default:
		return submodelV1
	}
}

// detectBioGripper: byteCount==2 means v1 (single-register response), 2*numRegs
// means v2 (full SN).
func (x *xArm) detectBioGripper(ctx context.Context) (detectedGripper, error) {
	const numRegs = 16
	c := x.gripperPreamble(false)
	c.params = binary.BigEndian.AppendUint16(c.params, bioGripperSNReg)
	c.params = binary.BigEndian.AppendUint16(c.params, numRegs)
	res, err := x.gripperSend(ctx, c)
	if err != nil {
		return unknownGripper(), err
	}
	const headerLen = 5
	if len(res.params) < headerLen {
		return unknownGripper(),
			fmt.Errorf("bio gripper response too short: %d (%v)", len(res.params), res.params)
	}
	byteCount := res.params[headerLen-1]
	switch byteCount {
	case 2:
		return detectedGripper{Kind: gripperKindBio, Version: "1"}, nil
	case 2 * numRegs:
		if len(res.params) < headerLen+int(byteCount) {
			return unknownGripper(),
				fmt.Errorf("bio gripper response truncated: got %d, want %d", len(res.params), headerLen+int(byteCount))
		}
		sn := strings.TrimRight(string(res.params[headerLen:headerLen+int(byteCount)]), "\x00 ")
		return detectedGripper{Kind: gripperKindBio, Version: "2 sn=" + sn}, nil
	default:
		return unknownGripper(),
			fmt.Errorf("bio gripper unexpected byte count: %d (%v)", byteCount, res.params)
	}
}

func probeGripper(ctx context.Context, a arm.Arm, kind gripperKind, logger logging.Logger) detectedGripper {
	x, err := utils.AssertType[*xArm](a)
	if err != nil {
		logger.Warnf("%s gripper detection skipped: %v", kind, err)
		return unknownGripper()
	}
	var d detectedGripper
	switch kind {
	case gripperKindStandard:
		d, err = x.detectStandardGripper(ctx)
	case gripperKindBio:
		d, err = x.detectBioGripper(ctx)
	case gripperKindVacuum:
		d, err = x.detectVacuumGripper(ctx)
	case gripperKindUnknown:
		return unknownGripper()
	default:
		logger.Warnf("gripper detection skipped: unrecognized kind %q", kind)
		return unknownGripper()
	}
	if err != nil {
		logger.Warnf("%s gripper detection failed: %v", kind, err)
		return d
	}
	logger.Infof("%s gripper detected (submodel=%q version=%q)", d.Kind, d.Submodel, d.Version)
	return d
}

func (x *xArm) detectVacuumGripper(ctx context.Context) (detectedGripper, error) {
	c := x.newCmd(regMap["VacuumState"])
	c.params = append(c.params, 0x09, 0x0A, 0x18)
	resp, err := x.send(ctx, c, true)
	if err != nil {
		return unknownGripper(), err
	}
	if len(resp.params) < 5 {
		return unknownGripper(),
			fmt.Errorf("vacuum gripper response too short: %d (%v)", len(resp.params), resp.params)
	}
	return detectedGripper{
		Kind:     gripperKindVacuum,
		Submodel: vacuumGripperSubmodel(x.detectedArm),
	}, nil
}

// vacuumGripperSubmodel infers hardware revision from the arm: TGPIO outputs
// 3/4 (v2) drive the 850 and xArm6/7 with submodel >= 1305; older xArms use
// TGPIO 0/1 (v1); Lite 6 has its own bus.
func vacuumGripperSubmodel(arm detectedArm) string {
	switch {
	case arm.Model == hardwareModelLite6:
		return submodelLite
	case arm.Model == hardwareModelXArm850:
		return submodelV2
	case arm.ArmTypeCode >= 1305:
		return submodelV2
	default:
		return submodelV1
	}
}
