# ros2_control_bridge

A `ros2_control` **SystemInterface** that talks to a motor driver over ROS
topics instead of a serial bus.

```
controllers ──▶ ros2_control_bridge ──▶ /<prefix>/<group>_cmd   (Float64MultiArray)
                        ▲
                        └────────────── /<prefix>/joint_states  (JointState)
```

**Nothing in it knows what robot it is on.** Groups, topics and interfaces all
come from the URDF, so one build serves every robot.

## Why a topic and not a serial port

A SystemInterface's `read()` and `write()` run inside controller_manager's
update loop. A blocking serial read there stalls every controller on the robot,
and a chain of hobby servos at 1 Mbaud is milliseconds per cycle, not
microseconds. Putting the bus behind a topic moves that latency out of the
control loop. The cost is a hop, and the loss of hard sync between a command and
the state it produced — which a servo bus does not offer anyway.

## Configuration

All of it lives in the `<ros2_control>` block of the URDF.

```xml
<ros2_control name="my_robot" type="system">
  <hardware>
    <plugin>ros2_control_bridge/TopicBridge</plugin>
    <param name="state_topic">/motor_manager/joint_states</param>
    <param name="cmd_topic_prefix">/motor_manager</param>
  </hardware>

  <joint name="joint_base">
    <param name="group">arm</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... four more arm joints ... -->

  <joint name="pan_joint">
    <param name="group">head</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

That produces two publishers — `/motor_manager/arm_cmd` with five values and
`/motor_manager/head_cmd` with two — and one subscriber.

### Hardware parameters

| param | default | |
|---|---|---|
| `state_topic` | `/motor_manager/joint_states` | `sensor_msgs/JointState` in |
| `cmd_topic_prefix` | `/motor_manager` | groups default to `<prefix>/<name>_cmd` |
| `group.<name>.topic` | — | override one group's topic |
| `default_group` | `default` | group for joints with no `group` param |
| `state_best_effort` | `true` | `false` for a reliable subscription |

### Per-joint parameters

| param | |
|---|---|
| `group` | which group — and therefore which topic and which array |

**The order of joints within a group is the order of values on that group's
topic.** It is the whole contract with the driver on the other end; reorder the
joints in the URDF and the wire format changes under it.

A group's interface is whatever its joints declare (`position` or `velocity`).
Mixing the two inside one group is an error: one array cannot be both. Declaring
two command interfaces on one joint is also an error — index *i* of the array
could then mean either.

Every joint exports **both** position and velocity state, whatever it commands,
because `joint_trajectory_controller` asks for velocity state on
position-commanded joints and refuses to configure without it.

Several `<ros2_control>` systems can each load their own instance — an
end-effector on the same bus as the arm, say. Each ignores joints it does not
own when reading the shared state topic.

Each instance names its node after its system: a block named
`mod101_harness_hardware` gets `/mod101_harness_hardware_bridge`. It used to be
the constant `ros2_control_topic_bridge` for all of them, which ROS 2 permits
and then warns about — `ros2 node list` showed one name twice with no way to
tell which was which, and anything addressing a node by name got whichever
answered first. Characters a node name cannot carry are folded to underscores.

## Startup interlock

A position group publishes nothing until **every joint the plugin owns has
appeared in a `JointState` message**. Then commands are seeded from the measured
positions and publishing begins, so the first message is the current pose rather
than a step to it.

Without that, the first `write()` would publish whatever the command interfaces
hold — `0.0` — and the arm would snap to its zero pose.

Velocity groups publish immediately: zero velocity is a safe command, and
withholding it would leave a wheel running through the wait.

## What this replaced

The first version had three hard-coded groups — `base` (velocity), `arm`
(position) and `camera` (position) — and decided which was which by testing
whether a joint's name contained the substring `"camera"`. On a robot whose
pan/tilt joints were called `hn_pan_joint` and `hn_tilt_joint`, both silently
joined the arm group and were published on the arm's topic, in the arm's array,
at indices the arm's driver owned. It also published a `base_cmd` array every
cycle, empty, on a robot with no wheels.

Its startup interlock waited for any arm joint to report a position more than
0.005 rad from zero. An arm parked at its zero pose never satisfied that, and
the bridge held position forever.

## Building

```bash
colcon build --packages-select ros2_control_bridge
```

Jazzy emits one deprecation warning for `on_init(const HardwareInfo&)`; the
replacement (`on_init(const HardwareComponentInterfaceParams&)`) is not present
across all Jazzy patch releases, so the older signature is deliberate.
