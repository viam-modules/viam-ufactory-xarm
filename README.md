# Viam UFactory xArm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for [UFactory's](https://www.ufactory.cc/) X-ARM 6, X-ARM 7, and LITE 6 collaborative arms.

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

This viam-xarm module is particularly useful in applications that require an xArm to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separate through your own code.

Navigate to the **CONFIGURE** tab of your machine’s page in [the Viam app](https://app.viam.com/). Click the **+** icon next to your machine part in the left-hand menu and select **Component**. Select the `arm` type, then search for and select the `arm / ufactory:xArm6`, `arm / ufactory:xArm7`, or `arm / ufactory:lite6` model, depending on your hardware model. Click **Add module**, then enter a name or use the suggested name for your arm and click **Create**.

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
| `host`                              | string  | **Required** | The IP address of the xArm.                                                                                      |
| `port`                              | string  | Optional     | The port at which the IP address accesses the xArm. The default is 502.                                          |
| `speed_degs_per_sec`                | float32 | Optional     | The rotational speed of the joints (must be greater than 3 and less than 180). The default is 50 degrees/second. |
| `acceleration_degs_per_sec_per_sec` | float32 | Optional     | The acceleration of joints in radians per second increase per second. The default is 100 degrees/second^2        |
| `bad-joints`                        | []int   | Optional     | Joints that cannot move                                                                                          |

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

## UFactory xArm Resources

The below documents will be useful for developers looking to contribute to this repository.

- [UFactory xArm User Manual](https://www.ufactory.cc/wp-content/uploads/2023/05/xArm-User-Manual-V2.0.0.pdf)
- [UFactory xArm Developer Manual](https://www.ufactory.cc/wp-content/uploads/2023/04/xArm-Developer-Manual-V1.10.0.pdf)
- [UFactory xArm Gripper User Manual](http://download.ufactory.cc/xarm/tool/Gripper%20User%20Manual.pdf?v=1594857600061)

## Note on xArm Studio

The arm itself runs xArm Studio. A developer should be able to use it through their web browser by going to the arm's IP address, port 18333.
