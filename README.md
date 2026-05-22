# Viam UFactory xArm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for [UFactory's](https://www.ufactory.cc/) X-ARM 6, X-ARM 7, X-ARM850, and LITE 6 collaborative arms.

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This viam-xarm module is particularly useful in applications that require an xArm to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separate through your own code.

Navigate to the **CONFIGURE** tab of your machine’s page in [the Viam app](https://app.viam.com/). Click the **+** icon next to your machine part in the left-hand menu and select **Component**. Select the `arm` type, then search for and select the `arm / ufactory:xArm6`, `arm / ufactory:xArm7`, `arm / ufactory:xArm850`, or `arm / ufactory:lite6` model, depending on your hardware model. Click **Add module**, then enter a name or use the suggested name for your arm and click **Create**.

> [!NOTE]
> Before configuring your xArm, you must [add a machine](https://docs.viam.com/fleet/machines/#add-a-new-machine).

## Configure your xArm

Copy and paste the following attributes into your JSON configuration:

```json
{
  "host": "0.0.0.0",
  "speed_degs_per_sec": 30
}
```

Edit the attributes as applicable.

### Attributes

The following attributes are available:

| Name                                | Type    | Inclusion    | Description                                                                                                      |
|-------------------------------------|---------|--------------|------------------------------------------------------------------------------------------------------------------|
| `host`                              | string  | **Required** | The IP address of the xArm. There is usually a sticker on the control box specifying the IP address. See below for detailed networking instructions.                                                                                      |
| `port`                              | string  | Optional     | The port at which the IP address accesses the xArm. The default is 502.                                          |
| `speed_degs_per_sec`                | float32 | Optional     | The rotational speed of the joints (must be greater than 3 and less than 180). The default is 50 degrees/second. |
| `acceleration_degs_per_sec_per_sec` | float32 | Optional     | The acceleration of joints in radians per second increase per second. The default is 100 degrees/second^2        |
| `collision_sensitivity`| int | Optional | Collision sensitivity range from 1-5. The larger the value, the smaller the force required to trigger the collision protection emergency stop. The default is 3.
| `bad-joints`                        | []int   | Optional     | Joints that cannot move                                                                                          |
| `motion`| string | Optional | The Motion Service to use for MoveToPosition API calls. Defaults to the builtin motion service. |
| `use_urdfs` | bool | Optional | When `true`, the arm's kinematic model is built from its URDF file instead of the default JSON kinematics. This attaches mesh-based collision geometries to each link, enabling more accurate collision checking. Default is `false`. |
| `mesh_decimation_ratios` | []float64 | Optional | Per-link mesh decimation ratios used when `use_urdfs` is `true`. Each value must be in the range [0, 1]. A value of 0.5 reduces a link's mesh to 50% of its original triangle count; lower values produce simpler (faster) collision geometries. The list length must match the number of joints (6 for xArm6/lite6, 7 for xArm7/uf850). If omitted when `use_urdfs` is `true`, defaults to 0.1 for every link. |
| `trajectory_generator` | object | Optional | Configuration for a [trajectory generator](#trajectory-generator) ML model service. When set, joint moves are planned by the service instead of the built-in interpolator. |
| `ufactory-studio-proxy` | bool | Optional | When `true`, starts a local reverse proxy to the arm's UFactory Studio web UI. See [UFactory Studio Proxy](#ufactory-studio-proxy). Default is `false`. |
| `ufactory-studio-proxy-port` | int | Optional | Local port for the Studio proxy. Default is `18333`. |

### Trajectory Generator

The `trajectory_generator` attribute connects the arm to an external service (such as [trajex](https://github.com/viamrobotics/trajex)) that performs time-optimal trajectory generation. When configured, all `MoveThroughJointPositions` calls are routed through the service instead of the built-in interpolator.

The service receives the arm's current position prepended to the requested waypoints, and returns a densely-sampled trajectory that respects the arm's velocity and acceleration limits. If the arm is already at the goal (within the deduplication tolerance), the service returns an empty result and no motion is commanded.

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
|---|---|---|---|
| `service` | string | **Required** | Name of the ML model service to use for trajectory generation. |
| `path_tolerance_delta_rads` | float64 | `0.1` | Maximum deviation from the straight-line path between waypoints, in radians. |
| `path_colinearization_ratio` | float64 | `0` (disabled by default) | Ratio used to merge nearly-collinear waypoints. Set to `0` to disable. |
| `waypoint_deduplication_tolerance_rads` | float64 | `0.001` | Waypoints closer than this value (in radians) are treated as duplicates and merged. |

The trajectory is sampled at the arm's configured `move_hz` frequency (default 100 Hz).

#### Connecting using macOS
The following steps can be followed when running viam-server on your mac.
1. Connect an ethernet cable between the Arm's control box and a USB-C hub/adapter connected to your mac.
1. Open "System Setting" -> "Network"
1. Click on the USB-C device within the "Other services" list
1. Ensure the device does not show "not connected", "not configured", or any status with a red indicator. If this is the case, there is a connection issue with the USB-C device, the ethernet cable, or the state of the Arm's control box. As soon as there is a yellow indicator, proceed with networking configuration below.
1. Click on "Details..."
1. Click on "TCP/IP"
1. Change "Configure IPv4" to "Manually".
1. Set the IP Address to any IP in the same subnet as the arm's IP address. There is usually a sticker on the control box specifying the IP. For example if the xArm IP is `192.168.1.2`, then an IP such as `192.168.1.10` should be set in your mac's system settings.
1. Set the Subnet Mask to `255.255.255.0`
1. Click OK to save the changes.
1. Your mac should now be able to connect to the xArm, and you can verify with `ping <host>` to the arm's IP address. Note that the IP address setup in your mac's networking settings is not the ping address and should not be used in Viam configuration. It was only set to establish a network to talk to the arm on the IP address that is specified on the control box.

#### Connecting using a Raspberry Pi
The following steps may need to be followed when running viam-server on your pi.


1. Connect an Ethernet cable from the xArm's control box to the Pi.
2. Open a terminal on the Pi.
3. Assign the Pi an IP address on the same subnet as the arm. For example, if the xArm’s IP is `192.168.1.2`, you can use:
```bash
sudo ip addr add 192.168.1.10/24 dev eth0
sudo ip link set eth0 up
```
4. Verify connectivity by pinging the xArm:
```bash
ping 192.168.1.2
```

### Using within a Frame System

If you are using your xArm in conjuction with other components it might be useful to add your arm to the frame system. You may do so by pasting the following in your config:

```json
"frame": {
    "parent": "world"
}
```

Then, for example, if you have a gripper attached to the arm's end effector you would have to add the following to the gripper's config for it to be understood by the frame system.

```json
{
  "name": "gripper",
  "namespace": "rdk",
  "type": "gripper",
  "model": "fake",
  "attributes": {},
  "frame": {
    "parent": "name of your xArm",
    "translation": {
      "x": 0,
      "y": 0,
      "z": 0
    },
    "geometry": {
      "type": "box",
      "x": 110,
      "y": 160,
      "z": 240,
      "translation": {
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    "orientation": {
      "type": "ov_degrees",
      "value": {
        "x": 0,
        "y": 0,
        "z": 1,
        "th": 0
      }
    }
  }
}
```

Edit the frame information as applicable.

### Using DoCommand

Below we provide examples of how a user may use Golang to use the `DoCommand`.

If you want to change the speed the arm operates:

```go
xArmComponent.DoCommand(context.Background(), map[string]interface{}{"set_speed": 50})
```

If you want to change the acceleration the arm operates at:

```go
xArmComponent.DoCommand(context.Background(), map[string]interface{}{"set_acceleration": 100})
```

If you want to change both the speed and acceleration:

```go
xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "set_speed":        50,
    "set_acceleration": 100,
})
```

If you want to get the current joint torques of the servo for each joint:

```go
load, err := xArmComponent.DoCommand(context.Background(), map[string]interface{}{"load": ""})
```

If you are using an UFactory gripper, you may use the `DoCommand` to manipulate it.
To fully open the gripper:

```go
xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "setup_gripper": true,
    "move_gripper":  850,
})
```

> [!NOTE] > `"setup_gripper": true` must be included in your request if you intend to manipulate the gripper

To close the gripper:

```go
xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "setup_gripper": true,
    "move_gripper":  0,
})
```

To get the current gripper position:

```go
resp, err := xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "get_gripper": true,
})
// resp["gripper_position"] contains the position (0-850)
```

To set the gripper speed (range 1-5000):

```go
xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "set_gripper_speed": 2000,
})
```

To get the current gripper speed:

```go
resp, err := xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "get_gripper_speed": true,
})
// resp["gripper_speed"] contains the speed
```

### Manual Mode (Teaching Mode)

You can put the arm into manual mode, which allows you to physically move the arm by hand. This is useful for teaching positions or manually positioning the arm for maintenance/storage.

To enter manual mode:

```go
resp, err := xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "enter_manual_mode": true,
})
```

To exit manual mode and return to normal operation:

```go
resp, err := xArmComponent.DoCommand(context.Background(), map[string]interface{}{
    "exit_manual_mode": true,
})
```

> [!NOTE]
> When in manual mode, the arm will be free to move by hand. Make sure the arm is properly supported and in a safe position before entering manual mode.

## UFactory xArm Resources

The below documents will be useful for developers looking to contribute to this repository.

- [UFactory xArm User Manual](https://www.ufactory.cc/wp-content/uploads/2023/05/xArm-User-Manual-V2.0.0.pdf)
- [UFactory xArm Developer Manual](https://www.ufactory.cc/wp-content/uploads/2023/04/xArm-Developer-Manual-V1.10.0.pdf)
- [UFactory xArm Gripper User Manual](http://download.ufactory.cc/xarm/tool/Gripper%20User%20Manual.pdf?v=1594857600061)
- [UFactory xArm Vacuum Gripper User Manual](https://static.generation-robots.com/media/xArm-Vacuum-Gripper-User-Manual-V1.6.1.pdf)


## UFactory Studio Proxy

The arm runs UFactory Studio, a web UI accessible at `http://<arm-ip>:18333`. When the machine running `viam-server` and the arm are on different subnets (e.g., arm on a direct Ethernet connection), the Studio UI may not be reachable from your browser.

Enabling `ufactory-studio-proxy` starts a local reverse proxy on the `viam-server` host that forwards requests to the arm's Studio port. This lets you access Studio at `http://<viam-server-ip>:<proxy-port>` without direct network access to the arm.

```json
{
  "host": "192.168.1.2",
  "ufactory-studio-proxy": true,
  "ufactory-studio-proxy-port": 18333
}
```

The proxy port defaults to `18333`. If that port is unavailable on the host, set `ufactory-studio-proxy-port` to a different value.

> [!CAUTION]
> The proxy listens on **all network interfaces** with **no authentication**. Anyone who can reach the proxy port on the `viam-server` host has full access to UFactory Studio, which can command the arm to move, change settings, and update firmware. Only enable this on trusted networks, or use firewall rules to restrict access to the proxy port.

## gripper

```jsonc
{
   "arm": "arm",           // required: name of the arm component
   "gripper_speed": 2000   // optional: default speed (1-5000)
}
```

| Name             | Type   | Inclusion    | Description                                                        |
|------------------|--------|--------------|--------------------------------------------------------------------|
| `arm`            | string | **Required** | The name of the arm component this gripper is attached to.         |
| `gripper_speed`  | int    | Optional     | Default gripper speed (1-5000). Applied on startup. If omitted, the gripper uses its firmware default. |

### DoCommand

```go
// Get the current position
resp, err := gripperComponent.DoCommand(context.Background(), map[string]interface{}{"get": true})
// resp["pos"] contains the position

// Move to a specific position
resp, err := gripperComponent.DoCommand(context.Background(), map[string]interface{}{"set": 500.0})
// resp["position"] contains the final position

// Set/get gripper speed (proxied to arm DoCommand)
resp, err := gripperComponent.DoCommand(context.Background(), map[string]interface{}{"set_gripper_speed": 2000})
resp, err := gripperComponent.DoCommand(context.Background(), map[string]interface{}{"get_gripper_speed": true})
```

## gripper lite
A two finger gripper compatible with the xarm lite6 model
```
{
   "arm" : "arm"
}
```

## vacuum gripper
The vacuum gripper only works if it has a wired connection to the not, not a contact connection
```
{
  "arm": "arm",
  "vacuum_length_mm" : 48
}
```

## vacuum gripper lite
The vacuum gripper commonly attached to the lite6.
```
{
  "arm": "arm",
  "vacuum_length_mm" : 48
}
```

