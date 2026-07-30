package arm

import (
	"context"
	"encoding/binary"
	"io"
	"math"
	"net"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/utils"
	"go.viam.com/test"
)

func TestParseFTSensorData(t *testing.T) {
	// Build a controller frame: 1 leading byte, then 6 little-endian float32 values.
	want := []float64{-0.9871726, -2.9230627, -18.356257, -0.0012335, -0.0913722, 0.0069847}
	params := make([]byte, 1+6*4)
	for i, v := range want {
		binary.LittleEndian.PutUint32(params[i*4+1:i*4+5], math.Float32bits(float32(v)))
	}

	got, err := parseFTSensorData(params)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(got), test.ShouldEqual, 6)
	for i := range want {
		test.That(t, got[i], test.ShouldAlmostEqual, want[i], 1e-4)
	}

	_, err = parseFTSensorData(make([]byte, 4))
	test.That(t, err, test.ShouldNotBeNil)
}

func TestTrajectoryStreamValidator(t *testing.T) {
	// point builds a trajectory point at time `d` with `dof` zeroed joint positions.
	point := func(d time.Duration, dof int) arm.TrajectoryPoint {
		return arm.TrajectoryPoint{Time: d, Positions: make([]referenceframe.Input, dof)}
	}
	// pointWithVel is `point` plus a declared `Velocities` entry on each joint.
	pointWithVel := func(d time.Duration, dof int, vel float64) arm.TrajectoryPoint {
		p := point(d, dof)
		vels := make([]float64, dof)
		for i := range vels {
			vels[i] = vel
		}
		p.Constraints = &arm.KinematicConstraints{Velocities: vels}
		return p
	}

	// The validator carries state across a stream, so each case feeds a whole sequence and asserts
	// where, if anywhere, the first rejection lands. A `failAtIdx` of -1 means the sequence is valid.
	for _, tc := range []struct {
		name      string
		points    []arm.TrajectoryPoint
		failAtIdx int
	}{
		{"valid increasing stream", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 6), point(20*time.Millisecond, 6)}, -1},
		{"first point must be t=0", []arm.TrajectoryPoint{point(5*time.Millisecond, 6)}, 0},
		{"time must strictly increase", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 6), point(10*time.Millisecond, 6)}, 2},
		{"time may not move backwards", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 6), point(5*time.Millisecond, 6)}, 2},
		{"dof must stay consistent", []arm.TrajectoryPoint{point(0, 6), point(10*time.Millisecond, 7)}, 1},
		{"positions must be non-empty", []arm.TrajectoryPoint{point(0, 0)}, 0},
		{"first point must be at rest", []arm.TrajectoryPoint{pointWithVel(0, 6, 0.5)}, 0},
		{"first point at rest with zero velocity is fine", []arm.TrajectoryPoint{pointWithVel(0, 6, 0), pointWithVel(10*time.Millisecond, 6, 0.5)}, -1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			v := newTrajectoryStreamValidator()
			firstErrIdx := -1
			for i, pt := range tc.points {
				if err := v.validate(pt); err != nil {
					firstErrIdx = i
					break
				}
			}
			test.That(t, firstErrIdx, test.ShouldEqual, tc.failAtIdx)
		})
	}
}

func TestCreateRawJointSteps1(t *testing.T) {
	var err error
	logger := logging.NewTestLogger(t)

	x := &xArm{
		speed:        utils.DegToRad(defaultSpeed),
		acceleration: utils.DegToRad(defaultAccel),
		moveHZ:       defaultMoveHz,
	}

	start := []float64{0, 0, 0, 0, 0, 0}
	x.model, err = MakeModelFrame("", ModelName6DOF, nil, start, false, nil, logger)
	test.That(t, err, test.ShouldBeNil)

	positions := [][]float64{{1, 0, 0, 0, 0, 1}}

	out, err := x.createRawJointSteps(start, positions, x.moveOptions(nil, nil))
	test.That(t, err, test.ShouldBeNil)

	minMoves := (1 / x.speed) * x.moveHZ
	test.That(t, len(out), test.ShouldBeGreaterThan, minMoves)
	test.That(t, len(out), test.ShouldBeLessThan, 20+minMoves)
}

func TestCreateRawJointStepsLowSpeed(t *testing.T) {
	var err error
	logger := logging.NewTestLogger(t)

	x := &xArm{
		speed:        utils.DegToRad(3),
		acceleration: utils.DegToRad(defaultAccel),
		moveHZ:       defaultMoveHz,
	}

	start := []float64{0, 0, 0, 0, 0, 0}
	x.model, err = MakeModelFrame("", ModelName6DOF, nil, start, false, nil, logger)
	test.That(t, err, test.ShouldBeNil)

	displacement := 1.0
	positions := [][]float64{{displacement, 0, 0, 0, 0, displacement}}

	out, err := x.createRawJointSteps(start, positions, x.moveOptions(nil, nil))
	test.That(t, err, test.ShouldBeNil)

	expected := (displacement / x.speed) * x.moveHZ
	// 15% band absorbs accel/decel ramp overhead.
	test.That(t, float64(len(out)), test.ShouldBeGreaterThan, 0.85*expected)
	test.That(t, float64(len(out)), test.ShouldBeLessThan, 1.15*expected)
}

func TestTgpioWord(t *testing.T) {
	test.That(t, tgpioWord(0, true), test.ShouldEqual, uint16(0x0101))  // v1 ON  pin0
	test.That(t, tgpioWord(1, false), test.ShouldEqual, uint16(0x0200)) // v1 ON  pin1
	test.That(t, tgpioWord(0, false), test.ShouldEqual, uint16(0x0100)) // v1 OFF pin0
	test.That(t, tgpioWord(1, true), test.ShouldEqual, uint16(0x0202))  // v1 OFF pin1
	test.That(t, tgpioWord(3, true), test.ShouldEqual, uint16(0x0404))  // v2 ON  pin3
	test.That(t, tgpioWord(4, false), test.ShouldEqual, uint16(0x0800)) // v2 ON  pin4
	test.That(t, tgpioWord(3, false), test.ShouldEqual, uint16(0x0400)) // v2 OFF pin3
	test.That(t, tgpioWord(4, true), test.ShouldEqual, uint16(0x0808))  // v2 OFF pin4
}

func TestTgpioDigitalParams_V1Regression(t *testing.T) {
	test.That(t, tgpioDigitalParams(0, true), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x80, 0x80, 0x43})
	test.That(t, tgpioDigitalParams(1, false), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x00, 0x00, 0x44})
	test.That(t, tgpioDigitalParams(0, false), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x00, 0x80, 0x43})
	test.That(t, tgpioDigitalParams(1, true), test.ShouldResemble,
		[]byte{0x09, 0x0A, 0x15, 0x00, 0x80, 0x00, 0x44})
}

func TestVacuumStateFromResponse(t *testing.T) {
	holdingV1 := []byte{0, 0, 0, 0, 0x01}
	holdingV2 := []byte{0, 0, 0, 0, 0x04}
	idle := []byte{0, 0, 0, 0, 0x00}

	got, err := vacuumStateFromResponse(holdingV1, connectionPlugin)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeTrue)

	got, err = vacuumStateFromResponse(holdingV2, connectionContact)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeTrue)

	got, err = vacuumStateFromResponse(holdingV1, connectionContact)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeFalse)

	got, err = vacuumStateFromResponse(idle, connectionPlugin)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeFalse)

	// Extra high bits set: masking (not equality) still reads as holding.
	got, err = vacuumStateFromResponse([]byte{0, 0, 0, 0, 0x05}, connectionPlugin)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, got, test.ShouldBeTrue)

	_, err = vacuumStateFromResponse([]byte{0, 0}, connectionPlugin)
	test.That(t, err, test.ShouldNotBeNil)
}

// fakeController is a stand-in xArm controller socket: it answers every command frame and records
// the register address each one targeted, so tests can assert the exact gripper-bus traffic a
// driver call produces.
type fakeController struct {
	ln net.Listener

	mu sync.Mutex
	// addrs holds the Modbus register address of each gripper frame received, in order.
	addrs []uint16
	// position is what a read of Fn702 (current position) reports back.
	position uint32
}

func newFakeController(t *testing.T) *fakeController {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	test.That(t, err, test.ShouldBeNil)

	f := &fakeController{ln: ln}
	go f.serve()
	t.Cleanup(func() { test.That(t, ln.Close(), test.ShouldBeNil) })
	return f
}

func (f *fakeController) addr() string { return f.ln.Addr().String() }

func (f *fakeController) seen() []uint16 {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]uint16(nil), f.addrs...)
}

func (f *fakeController) reset() {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.addrs = nil
}

func (f *fakeController) serve() {
	for {
		conn, err := f.ln.Accept()
		if err != nil {
			return
		}
		go f.handle(conn)
	}
}

func (f *fakeController) handle(conn net.Conn) {
	defer conn.Close() //nolint:errcheck
	for {
		header := make([]byte, 7)
		if _, err := io.ReadFull(conn, header); err != nil {
			return
		}
		params := make([]byte, int(binary.BigEndian.Uint16(header[4:6]))-1)
		if _, err := io.ReadFull(conn, params); err != nil {
			return
		}
		if _, err := conn.Write(f.respond(header, params)); err != nil {
			return
		}
	}
}

// respond builds the reply frame. Gripper frames are [hostID, gripperID, fn, addrHi, addrLo, ...];
// a read of Fn702 gets the current position, everything else gets a bare success state byte.
func (f *fakeController) respond(header, params []byte) []byte {
	resp := cmd{
		tid:    binary.BigEndian.Uint16(header[0:2]),
		prot:   binary.BigEndian.Uint16(header[2:4]),
		reg:    header[6],
		params: []byte{0x00}, // state byte: no error, no warning
	}
	if resp.reg != regMap["GripperControl"] || len(params) < 5 {
		return resp.bytes()
	}

	addr := binary.BigEndian.Uint16(params[3:5])
	f.mu.Lock()
	f.addrs = append(f.addrs, addr)
	position := f.position
	f.mu.Unlock()

	if params[2] == 0x03 && addr == 0x0702 {
		resp.params = append(resp.params, 0x09, 0x08, 0x03, 0x04)
		resp.params = binary.BigEndian.AppendUint32(resp.params, position)
	}
	return resp.bytes()
}

func newTestArm(t *testing.T, cmdAddr, gripperAddr string) *xArm {
	t.Helper()
	logger := logging.NewTestLogger(t)
	x := &xArm{
		logger:  logger,
		cmdConn: newModbusConn(cmdAddr, logger, nil),
	}
	x.gripperConn = newModbusConn(gripperAddr, logger, nil)
	x.gripperControlMode.Store(true)
	return x
}

// Gripper-bus register addresses, for readability in the assertions below.
const (
	regGripperEnable      = 0x0100
	regGripperMode        = 0x0101
	regGripperSpeed       = 0x0303
	regGripperControlMode = 0x0C00
)

// A configured gripper speed must survive later gripper operations. setupGripper runs before every
// move, and clearing the FnCxx control-mode block resets Fn303, so an unconditional clear there
// silently discards the speed.
func TestSetupGripperPreservesSpeed(t *testing.T) {
	ctx := context.Background()
	controller := newFakeController(t)
	x := newTestArm(t, controller.addr(), controller.addr())

	// First setup clears the control mode once, in case a previous process left it enabled.
	test.That(t, x.setupGripper(ctx), test.ShouldBeNil)
	test.That(t, controller.seen(), test.ShouldResemble,
		[]uint16{regGripperControlMode, regGripperEnable, regGripperMode})

	controller.reset()
	test.That(t, x.setGripperSpeed(ctx, 500), test.ShouldBeNil)
	test.That(t, controller.seen(), test.ShouldResemble, []uint16{regGripperSpeed})

	// Every later setup leaves the control-mode block, and so the speed, alone.
	controller.reset()
	test.That(t, x.setupGripper(ctx), test.ShouldBeNil)
	test.That(t, x.setupGripper(ctx), test.ShouldBeNil)
	test.That(t, controller.seen(), test.ShouldResemble, []uint16{
		regGripperEnable, regGripperMode,
		regGripperEnable, regGripperMode,
	})
}

// graspWithTorque leaves the control mode enabled, so the next setup has to clear it — and then put
// the configured speed back, because the clear resets it.
func TestSetupGripperRestoresSpeedAfterTorqueGrasp(t *testing.T) {
	ctx := context.Background()
	controller := newFakeController(t)
	controller.position = 100 // report the grasp target so waitForGripper returns promptly
	x := newTestArm(t, controller.addr(), controller.addr())

	test.That(t, x.setGripperSpeed(ctx, 500), test.ShouldBeNil)

	controller.reset()
	test.That(t, x.graspWithTorque(ctx, 3000, 50, 100, time.Second), test.ShouldBeNil)
	test.That(t, controller.seen()[:2], test.ShouldResemble,
		[]uint16{regGripperControlMode, regGripperControlMode})

	controller.reset()
	test.That(t, x.setupGripper(ctx), test.ShouldBeNil)
	test.That(t, controller.seen(), test.ShouldResemble,
		[]uint16{regGripperControlMode, regGripperSpeed, regGripperEnable, regGripperMode})
}

// Gripper-bus traffic belongs on the dedicated gripper socket. Splitting it across both sockets
// lets the controller reorder a control-mode write past a speed write on the shared RS485 bus.
func TestGripperTrafficStaysOnGripperConn(t *testing.T) {
	ctx := context.Background()
	cmdController := newFakeController(t)
	gripperController := newFakeController(t)
	gripperController.position = 100
	x := newTestArm(t, cmdController.addr(), gripperController.addr())

	test.That(t, x.setupGripper(ctx), test.ShouldBeNil)
	test.That(t, x.setGripperSpeed(ctx, 500), test.ShouldBeNil)
	test.That(t, x.graspWithTorque(ctx, 3000, 50, 100, time.Second), test.ShouldBeNil)

	test.That(t, cmdController.seen(), test.ShouldBeEmpty)
	test.That(t, gripperController.seen(), test.ShouldNotBeEmpty)
}
