# airport_clamp_driver

ROS1 Noetic driver for the STM32 dual-axis airport clamp.

## Network

The STM32 W5500 is configured as a TCP server at `192.168.8.103:5000`.
The node binds the industrial PC control interface at `192.168.8.102` and
only accepts a TCP peer with that address. Commands and responses are ASCII
lines terminated by CRLF.

```
GRIP\r\n       # close both axes to the configured end points
RELEASE\r\n    # open both axes to the configured start points
STOP\r\n
CLEAR\r\n
STATUS\r\n
ENDPOINTS 1 275.000 130.000\r\n
```

The network layer stops the motors when the TCP connection is lost, when no
request is received for three seconds, or when an invalid command is received.
It does not replay a motion command after reconnecting.

## ROS interface

Launch with:

```
source /opt/ros/noetic/setup.bash
source ~/workspace/ros1_vehicles/devel/setup.bash
roslaunch airport_clamp_driver airport_clamp.launch
```

Build into the workspace's existing `build` and `devel` directories:

```
cd ~/workspace/ros1_vehicles
source /opt/ros/noetic/setup.bash
catkin_make --pkg airport_clamp_driver -j2
```

`/clamp_command` uses `std_msgs/UInt8`: `0` is RELEASE and `1` is GRIP.
The node publishes `/clamp_status` (`std_msgs/String`),
`/clamp_distance_mm` (`std_msgs/Float32MultiArray`, axis 1 then axis 2), and
`/clamp_connected` (`std_msgs/Bool`).

Private services are `/airport_clamp/stop`, `/airport_clamp/clear`, and
`/airport_clamp/set_endpoints` (`SetEndpoints`: axis, start_mm, end_mm).

The drone interface uses the ROS topic pair `/clamp_command` and `/clamp_state` through `ros1_bridge`. See `docs/drone_radio_interface.md` for the ROS1/ROS2 mapping.

## Host-owned thresholds

Edit `config/clamp.yaml` on the industrial PC:

```yaml
axis1_release_mm: 275.0
axis1_grip_mm: 130.0
axis2_release_mm: 275.0
axis2_grip_mm: 130.0
```

All four values are absolute sensor distances in mm (0.1 mm resolution).
For the confirmed sensor mounting, grip distance must be smaller than release
distance; separation must exceed 1 mm to accommodate the +/-0.5 mm tolerance.

On startup and every reconnect the node sends STOP, sets both axes' endpoints,
and verifies the reported values. Only then does `/clamp_connected` become true
and external `/clamp_command` messages take effect. Setting thresholds does not
start either motor. The existing STM32 network firmware supports this interface;
no MCU firmware download is needed for this change.

Restart the node after editing this YAML. For a different command topic:

```bash
roslaunch airport_clamp_driver airport_clamp.launch command_topic:=/your_command
```

For a runtime update while both axes are idle:

```bash
rosservice call /airport_clamp/set_endpoints '{axis: 1, start_mm: 275.0, end_mm: 130.0}'
```

`start_mm` means release distance, `end_mm` means grip distance. After acceptance
and readback verification, the node updates its ROS parameters and reapplies these
values on TCP reconnect. Service changes do not rewrite the YAML; edit the YAML
to retain them after restarting roslaunch. Direct `rosparam set` does not apply
thresholds to the MCU; use this service or restart with the edited YAML.
