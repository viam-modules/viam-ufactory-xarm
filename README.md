# Viam UFactory xArm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for [UFactory's](https://www.ufactory.cc/) family of collaborative arms.

This viam-xarm module is particularly useful in applications that require an xArm to be operated in conjunction with other resources (such as cameras, sensors, actuators, CV) offered by the [Viam Platform](https://www.viam.com/) and/or separate through your own code.

> [!NOTE]
> For more information on modules, see [Modular Resources](https://docs.viam.com/registry/#modular-resources).

## Configure your xArm

> [!NOTE]
> Before configuring your xArm, you must [add a machine](https://docs.viam.com/fleet/machines/#add-a-new-machine).

Navigate to the **CONFIGURE** tab of your machine’s page in [the Viam app](https://app.viam.com/). Click the **+** icon next to your machine part in the left-hand menu and select **Component**. Select the `arm` type, then search for and select the `arm / viam-UFactory-xarm` model. Click **Add module**, then enter a name or use the suggested name for your arm and click **Create**.

On the new component panel, copy and paste the following attribute template into your arm’s attributes field:

```json
{
  "host": "0.0.0.0",
  "speed_degs_per_sec": 30,
}
```

Edit the attributes as applicable.

> [!NOTE]
> For more information, see [Configure a Machine](https://docs.viam.com/build/configure/).

## Attributes

The following attributes are available:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `host` | string | **Required** | The IP address of the xArm.  |
| `speed_degs_per_sec` | float64 | **Optional** | The rotational speed of the joints (must be greater than 3 and less than 180). The default is 50 degrees/second.  |

## Using within a Frame System

If you are using your xArm in conjuction with other components it might be useful to add your arm to the frame system. You may do so by pasting the following in your config:
```json
"frame": {
    "parent": "world",
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

## UFactory xArm Resources
The below documents will be useful for developers looking to contribute to this repository.
* [UFactory xArm User Manual](https://www.ufactory.cc/wp-content/uploads/2023/05/xArm-User-Manual-V2.0.0.pdf)
* [UFactory xArm Developer Manual](https://www.ufactory.cc/wp-content/uploads/2023/04/xArm-Developer-Manual-V1.10.0.pdf)
* [UFactory xArm Gripper User Manual](http://download.ufactory.cc/xarm/tool/Gripper%20User%20Manual.pdf?v=1594857600061)

## Note on xArm Studio

The arm itself runs xArm Studio. A developer should be able to use it through their web browser by going to the arm's IP address, port 18333.
