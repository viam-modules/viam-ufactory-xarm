package arm

import (
	"context"
	"encoding/binary"
	"io"
	"net"
	"reflect"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
)

func TestClearJointLimitErrorDoCommand(t *testing.T) {
	for _, tc := range []struct {
		name                   string
		fault, warning, motion byte
		cleared, failed        bool
	}{
		{"joint_limit", 23, 0, 4, true, false},
		{"healthy", 0, 0, 2, false, false},
		{"collision", 31, 0, 4, false, true},
		{"estop", 1, 0, 4, false, true},
		{"servo", 11, 0, 4, false, true},
		{"unknown", 255, 0, 4, false, true},
		{"warning", 23, 11, 4, false, true},
		{"moving", 23, 0, 1, false, true},
		{"after_motion_stop", 23, 0, 3, true, false},
		{"fault_changed", 23, 0, 4, false, true},
		{"clear_failed", 23, 0, 4, false, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			client, server := net.Pipe()
			t.Cleanup(func() { client.Close(); server.Close() })
			done := make(chan []byte, 1)
			go func() {
				var requests []byte
				defer func() { done <- requests }()
				fault := tc.fault
				for {
					header := make([]byte, 7)
					if _, err := io.ReadFull(server, header); err != nil {
						return
					}
					payload := make([]byte, int(binary.BigEndian.Uint16(header[4:6]))-1)
					if _, err := io.ReadFull(server, payload); err != nil {
						return
					}
					requests = append(requests, header[6])
					state := byte(0)
					if fault != 0 {
						state |= errorState
					}
					if tc.warning != 0 {
						state |= warningState
					}
					response := cmd{tid: binary.BigEndian.Uint16(header), prot: 2, reg: header[6]}
					switch header[6] {
					case regMap["SetState"]:
						response.params = []byte{state}
					case regMap["GetError"]:
						response.params = []byte{state, fault, tc.warning}
					case regMap["GetState"]:
						response.params = []byte{state, tc.motion}
						if tc.name == "fault_changed" {
							fault = 31
						}
					case regMap["ClearError"]:
						if tc.name != "clear_failed" {
							fault = 0
						}
						response.params = []byte{0}
					default:
						server.Close()
						return
					}
					if _, err := server.Write(response.bytes()); err != nil {
						return
					}
				}
			}()
			x := &xArm{cmdConn: &modbusConn{conn: client, logger: logging.NewTestLogger(t)}, opMgr: operation.NewSingleOperationManager()}
			if tc.name == "after_motion_stop" {
				if err := x.Stop(context.Background(), nil); err == nil {
					t.Fatal("Stop should preserve the controller fault")
				}
			}
			result, err := x.DoCommand(context.Background(), map[string]any{"clear_joint_limit_error": true})
			client.Close()
			requests := <-done
			if (err != nil) != tc.failed || result["cleared"] != tc.cleared {
				t.Fatalf("result=%v err=%v requests=%v", result, err, requests)
			}
			want := []byte{regMap["GetError"]}
			if tc.name == "after_motion_stop" {
				want = append([]byte{regMap["SetState"], regMap["GetError"]}, want...)
			}
			if tc.fault == 23 && tc.warning == 0 {
				want = append(want, regMap["GetState"])
			}
			if tc.fault == 23 && tc.warning == 0 && (tc.motion == 4 || tc.motion == 3) {
				want = append(want, regMap["GetError"])
			}
			if tc.cleared || tc.name == "clear_failed" {
				want = append(want, regMap["ClearError"], regMap["GetError"])
			}
			if !reflect.DeepEqual(requests, want) {
				t.Fatalf("controller commands=%v want=%v", requests, want)
			}
			t.Logf("fault=%d cleared=%v controller_registers=%v", tc.fault, result["cleared"], requests)
		})
	}
}

func TestClearJointLimitErrorRejectsBeforeControllerAccess(t *testing.T) {
	x := &xArm{opMgr: operation.NewSingleOperationManager()}
	for _, command := range []map[string]any{
		{"clear_joint_limit_error": false}, {"clear_joint_limit_error": "true"},
		{"clear_joint_limit_error": true, "clear_error": true},
	} {
		if _, err := x.DoCommand(context.Background(), command); err == nil {
			t.Fatal("accepted malformed command")
		}
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if _, err := x.DoCommand(ctx, map[string]any{"clear_joint_limit_error": true}); err == nil {
		t.Fatal("accepted cancellation")
	}
	x.started.Store(int32(manualMode))
	if _, err := x.DoCommand(context.Background(), map[string]any{"clear_joint_limit_error": true}); err == nil {
		t.Fatal("accepted manual mode")
	}
}
