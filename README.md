
# Viam UFactory xArm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for [UFactory's](https://www.ufactory.cc/) xArm 6, xArm 7, xArm 850, and Lite 6 collaborative arms.

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This module is particularly useful in applications that require an xArm to be operated alongside other resources (cameras, sensors, actuators, CV pipelines) offered by the [Viam Platform](https://www.viam.com/).

## Contents

- [Getting Started](#getting-started)
- [Configure your xArm](#configure-your-xarm)
  - [Attributes](#attributes)
  - [Networking](#networking)
  - [Trajectory Generator](#trajectory-generator)
  - [Using within a Frame System](#using-within-a-frame-system)
- [Error Handling](#error-handling)
- [DoCommand Reference](#docommand-reference)
- [UFactory Studio Proxy](#ufactory-studio-proxy)
- [Gripper](#gripper)
- [Gripper Lite](#gripper-lite)
- [Vacuum Gripper](#vacuum-gripper)
- [Vacuum Gripper Lite](#vacuum-gripper-lite)
- [UFactory xArm Resources](#ufactory-xarm-resources)

## Getting Started

1. [Add a machine](https://docs.viam.com/set-up-a-machine/first-machine/) in the Viam app.
2. Navigate to the **CONFIGURE** tab of your machine's page.
3. Click the **+** icon next to your machine part and select **Configuration block**.
4. Search for and select `ufactory/xArm6`, `ufactory/xArm7`, `ufactory/xArm850`, or `ufactory/lite6`.
5. Click **Add to machine**, enter a name for your arm and click **Add to machine** once more.
6. Set the `host` attribute to your arm's IP address (found on the sticker on the control box).

## Configure your xArm

Copy and paste the following attributes into your JSON configuration:

```json
{
  "host": "0.0.0.0",
  "speed_degs_per_sec": 30
}
```

### Attributes

| Name | Type | Inclusion | Default | Description |
|------|------|-----------|---------|-------------|
| `host` | string | **Required** | — | IP address of the xArm. Found on the sticker on the control box. See [Networking](#networking) below. |
| `port` | int | Optional | `502` | TCP port for the arm's Modbus interface. |
| `speed_degs_per_sec` | float32 | Optional | `60` | Joint speed in degrees/second. Must be between `3` and `180`. |
| `acceleration_degs_per_sec_per_sec` | float32 | Optional | `381.67` | Joint acceleration in degrees/second². Must not exceed `1145`. |
| `collision_sensitivity` | int | Optional | `3` | Collision detection sensitivity from `0` (off) to `5`. Higher values trigger the emergency stop with less force. |
| `bad-joints` | []int | Optional | — | List of joint indices that cannot move. The arm will be configured to lock those joints at their current position on startup. |
| `motion` | string | Optional | `builtin` | Name of the motion service to use for `MoveToPosition` API calls. |
| `use_urdfs` | bool | Optional | `false` | When `true`, builds the kinematic model from the arm's URDF file, attaching mesh-based collision geometries to each link for more accurate collision checking. |
| `mesh_decimation_ratios` | []float64 | Optional | `0.1` per link | Per-link mesh simplification ratios when `use_urdfs` is `true`. Each value must be in `[0, 1]`; `0.5` reduces a link to 50% of its original triangle count. List length must match the number of joints (6 for xArm6/Lite6, 7 for xArm7/xArm850). |
| `trajectory_generator` | object | Optional | — | Configuration for an external [trajectory generator](#trajectory-generator) ML model service. |
| `ufactory-studio-proxy` | bool | Optional | `false` | When `true`, starts a local reverse proxy to the arm's UFactory Studio web UI. See [UFactory Studio Proxy](#ufactory-studio-proxy). |
| `ufactory-studio-proxy-port` | int | Optional | `18333` | Local port for the Studio proxy. |

### Networking

#### Connecting using macOS

1. Connect an Ethernet cable between the arm's control box and a USB-C hub connected to your Mac.
2. Open **System Settings** → **Network**.
3. Click the USB-C device under "Other services." Ensure it shows a green indicator (not red/disconnected).
4. Click **Details...** → **TCP/IP**.
5. Set **Configure IPv4** to **Manually**.
6. Set the IP address to any address on the same subnet as the arm (e.g., if the arm is `192.168.1.2`, use `192.168.1.10`).
7. Set the Subnet Mask to `255.255.255.0` and click OK.
8. Verify with `ping <arm-ip>`. Use the arm's IP (from the control box sticker) in your Viam config — not the address you assigned to your Mac.

#### Connecting using Linux

1. Connect an Ethernet cable from the arm's control box to your machine.
2. Assign your machine an IP address on the same subnet as the arm:
```bash
sudo ip addr add 192.168.1.10/24 dev eth0
sudo ip link set eth0 up
```
3. Verify connectivity:
```bash
ping 192.168.1.2
```

> [!NOTE]
> Replace `eth0` with your actual Ethernet interface name (find it with `ip link show`) and replace `192.168.1.10`/`192.168.1.2` with addresses appropriate for your arm's subnet.

### Trajectory Generator

The `trajectory_generator` attribute connects the arm to an external ML model service (such as [trajex](https://github.com/viamrobotics/trajex)) for time-optimal trajectory generation. When configured, all `MoveThroughJointPositions` calls are routed through the service instead of the built-in interpolator.

```json
{
  "trajectory_generator": {
    "service": "my-trajex-service",
    "path_tolerance_delta_rads": 0.1,
    "waypoint_deduplication_tolerance_rads": 0.001
  }
}
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `service` | string | **Required** | Name of the ML model service. |
| `path_tolerance_delta_rads` | float64 | `0.1` | Maximum deviation from the straight-line path between waypoints, in radians. |
| `path_colinearization_ratio` | float64 | `0` (disabled) | Ratio used to merge nearly-collinear waypoints. |
| `waypoint_deduplication_tolerance_rads` | float64 | `0.001` | Waypoints closer than this value (in radians) are treated as duplicates and merged. |

### Using within a Frame System

To use your xArm alongside other components, add it to the frame system:

```json
"frame": {
    "parent": "world"
}
```

For an attached gripper, set its parent to the arm's name:

```json
{
  "name": "gripper",
  "api": "rdk:component:gripper",
  "model": "viam:ufactory:gripper",
  "attributes": {},
  "frame": {
    "parent": "my-xarm",
    "translation": { "x": 0, "y": 0, "z": 0 },
    "geometry": {
      "type": "box",
      "x": 110, "y": 160, "z": 240,
      "translation": { "x": 0, "y": 0, "z": 0 }
    },
    "orientation": {
      "type": "ov_degrees",
      "value": { "x": 0, "y": 0, "z": 1, "th": 0 }
    }
  }
}
```

## Error Handling

When a collision or other fault occurs, the arm enters an error state and will not accept motion commands. The driver attempts to clear transient errors automatically on each command. A collision (overcurrent error) requires manual intervention:

1. Remove any obstacles causing the collision.
2. Clear the error using `DoCommand` (Go):
```go
xArmComponent.DoCommand(context.Background(), map[string]interface{}{"clear_error": true})
```
Or via Python SDK:
```python
await arm.do_command({"clear_error": True})
```
3. The arm will return to normal operation automatically.

To inspect current arm state or error codes:
```go
// Get current arm state
resp, _ := xArmComponent.DoCommand(context.Background(), map[string]interface{}{"get_state": true})
// resp["state"] contains raw state bytes

// Get current error code
resp, _ := xArmComponent.DoCommand(context.Background(), map[string]interface{}{"get_error": true})
// resp["error info"] contains raw error bytes
```

## DoCommand Reference

The following commands are available via `DoCommand` on the arm component.

### Speed and Acceleration

**Go:**
```go
// Set speed (degrees/second)
xArmComponent.DoCommand(ctx, map[string]interface{}{"set_speed": 50.0})

// Set acceleration (degrees/second²)
xArmComponent.DoCommand(ctx, map[string]interface{}{"set_acceleration": 100.0})

// Set both
xArmComponent.DoCommand(ctx, map[string]interface{}{
    "set_speed":        50.0,
    "set_acceleration": 100.0,
})
```

**Python:**
```python
await arm.do_command({"set_speed": 50.0, "set_acceleration": 100.0})
```

### Joint Torques

```go
resp, err := xArmComponent.DoCommand(ctx, map[string]interface{}{"load": ""})
// resp["load"] contains a []float64 of per-joint torque values
```

### UFactory Gripper Control (via arm DoCommand)

> [!NOTE]
> `"setup_gripper": true` must be included in any gripper move command.

```go
// Open fully
xArmComponent.DoCommand(ctx, map[string]interface{}{
    "setup_gripper": true,
    "move_gripper":  850.0,
})

// Close
xArmComponent.DoCommand(ctx, map[string]interface{}{
    "setup_gripper": true,
    "move_gripper":  0.0,
})

// Get position (returns 0–850)
resp, _ := xArmComponent.DoCommand(ctx, map[string]interface{}{"get_gripper": true})
// resp["gripper_position"]

// Set speed (range 1–5000)
xArmComponent.DoCommand(ctx, map[string]interface{}{"set_gripper_speed": 2000.0})

// Get speed
resp, _ := xArmComponent.DoCommand(ctx, map[string]interface{}{"get_gripper_speed": true})
// resp["gripper_speed"]
```

### Vacuum Gripper Control (via arm DoCommand)

```go
// Activate suction (grab)
xArmComponent.DoCommand(ctx, map[string]interface{}{"grab_vacuum": true})

// Release (open)
xArmComponent.DoCommand(ctx, map[string]interface{}{"open_vacuum": true})

// Get suction state
resp, _ := xArmComponent.DoCommand(ctx, map[string]interface{}{"get_vacuum_state": true})
// resp["vacuum_state"] is a bool
```

### Manual Mode (Teaching Mode)

Manual mode puts the arm into zero-gravity mode, allowing free movement by hand with gravity compensation active. Servos remain engaged — do not disable them.

```go
// Enter manual mode
xArmComponent.DoCommand(ctx, map[string]interface{}{"enter_manual_mode": true})

// Exit manual mode and return to normal operation
xArmComponent.DoCommand(ctx, map[string]interface{}{"exit_manual_mode": true})
```

> [!CAUTION]
> Ensure the arm's payload and mounting orientation are correctly configured before entering manual mode, or gravity compensation will be inaccurate and the arm may drift.

## UFactory Studio Proxy

The arm hosts UFactory Studio at `http://<arm-ip>:18333`. When viam-server and the arm are on different subnets (e.g., direct Ethernet connection), Studio may not be reachable from your browser.

Enable `ufactory-studio-proxy` to start a local reverse proxy on the viam-server host:

```json
{
  "host": "192.168.1.2",
  "ufactory-studio-proxy": true,
  "ufactory-studio-proxy-port": 18333
}
```

Access Studio at `http://<viam-server-ip>:18333`. If port 18333 is in use, set `ufactory-studio-proxy-port` to a different value.

> [!CAUTION]
> The proxy listens on all interfaces with no authentication. Anyone who can reach that port has full access to UFactory Studio, which can command the arm to move, change settings, and update firmware. Only enable this on trusted networks, or restrict access with firewall rules.

## Gripper

The standard two-finger gripper for xArm6/xArm7.

```json
{
  "arm": "my-xarm",
  "gripper_speed": 2000
}
```

| Name | Type | Inclusion | Description |
|------|------|-----------|-------------|
| `arm` | string | **Required** | Name of the arm component this gripper is attached to. |
| `gripper_speed` | int | Optional | Default speed on startup (1–5000). Uses firmware default if omitted. |

### DoCommand

```go
// Get current position (0–850)
resp, _ := gripperComponent.DoCommand(ctx, map[string]interface{}{"get": true})
// resp["pos"]

// Move to a specific position (0–850)
resp, _ := gripperComponent.DoCommand(ctx, map[string]interface{}{"set": 500.0})
// resp["position"]

// Set/get speed (proxied to arm)
gripperComponent.DoCommand(ctx, map[string]interface{}{"set_gripper_speed": 2000.0})
resp, _ := gripperComponent.DoCommand(ctx, map[string]interface{}{"get_gripper_speed": true})
```

## Gripper Lite

Two-finger gripper for the Lite 6.

```json
{
  "arm": "my-xarm"
}
```

### DoCommand

```go
// Open
gripperLiteComponent.DoCommand(ctx, map[string]interface{}{"gripper_lite_action": "open"})

// Close
gripperLiteComponent.DoCommand(ctx, map[string]interface{}{"gripper_lite_action": "close"})

// Stop
gripperLiteComponent.DoCommand(ctx, map[string]interface{}{"gripper_lite_action": "stop"})

// Check if holding something
resp, _ := gripperLiteComponent.DoCommand(ctx, map[string]interface{}{"gripper_lite_action": "is_closed"})
// resp["gripper_lite_action"]["is_closed"] is a bool
```

## Vacuum Gripper

For use with the standard xArm vacuum gripper. Requires a wired connection to the arm controller (not a contact connection).

```json
{
  "arm": "my-xarm",
  "vacuum_length_mm": 48
}
```

## Vacuum Gripper Lite

Vacuum gripper for the Lite 6.

```json
{
  "arm": "my-xarm",
  "vacuum_length_mm": 48
}
```

## Force Torque Sensor

Model `viam:ufactory:ft_sensor` exposes the UFactory wrist-mounted 6-axis
Force/Torque sensor as a Viam `sensor`. It depends on a configured xArm and reads
through the arm's controller connection. Requires controller firmware >= 1.8.3.

### Configuration

```json
{
  "arm": "my-xarm",
  "enable_on_start": false
}
```

| Attribute         | Type   | Required | Description |
|-------------------|--------|----------|-------------|
| `arm`             | string | yes      | Name of the xArm this sensor is attached to. |
| `enable_on_start` | bool   | no       | Enable the sensor when the resource starts. Default `false`. Leave `false` if you enabled the sensor in UFactory Studio (that setting persists on the controller); set `true` if you did not, so readings work without a manual `DoCommand`. |

### Readings

`GetReadings` returns the same keys and units as the Universal Robots F/T sensor:

```json
{ "Fx_N": -0.987, "Fy_N": -2.923, "Fz_N": -18.356,
  "TRx_Nm": -0.0012, "TRy_Nm": -0.0914, "TRz_Nm": 0.00698 }
```

Forces (`F*_N`) are in newtons; torques (`TR*_Nm`) are in newton-metres.

### DoCommand

| Command | Effect |
|---------|--------|
| `{"enable": true}` | Enable the sensor (required before taring/reading). |
| `{"enable": false}` | Disable the sensor. |
| `{"tare": true}` | Zero the sensor at the current reading. Hold the arm stationary at the unloaded reference pose first. |

## UFactory xArm Resources

- [UFactory xArm User Manual](https://www.ufactory.cc/wp-content/uploads/2023/05/xArm-User-Manual-V2.0.0.pdf)
- [UFactory xArm Developer Manual](https://www.ufactory.cc/wp-content/uploads/2023/04/xArm-Developer-Manual-V1.10.0.pdf)
- [UFactory xArm Gripper User Manual](http://download.ufactory.cc/xarm/tool/Gripper%20User%20Manual.pdf?v=1594857600061)
- [UFactory xArm Vacuum Gripper User Manual](https://static.generation-robots.com/media/xArm-Vacuum-Gripper-User-Manual-V1.6.1.pdf)
