# GEACX1 YTJ-28PUL ROS1 driver

This package drives the YTJ-28PUL pulse/direction inputs from the GEACX1 GPIO
sysfs nodes:

- `GPIO_OUTPUT1` -> `P+` (pulse)
- `GPIO_OUTPUT2` -> `D+` (direction)
- GEACX1 `GND` -> `P-` and `D-`

The YTJ-28PUL manual specifies 3.3-5 V logic and a 100-20000 Hz pulse input.
The node enforces this frequency range. It always leaves the pulse output low
when idle, stopped, shutting down, or handling an error, and restores both GPIO
outputs low when the node exits.

## Build

```bash
cd ~/workspace/ros1_vehicles
source /opt/ros/noetic/setup.bash
catkin_make --pkg stepper_gpio_driver
source devel/setup.bash
```

The `nvidia` user must belong to the `gpio` group. Check with `id`; the GEACX1
used for this package already has that membership.

## Configure the pulse conversion

Set `pulses_per_revolution` to the subdivision selected by S1/S2/S3 on the
YTJ-28PUL:

| S1 | S2 | S3 | Pulses/revolution |
| --- | --- | --- | ---: |
| ON | ON | ON | 200 |
| ON | ON | OFF | 3200 |
| ON | OFF | ON | 800 |
| ON | OFF | OFF | 12800 |
| OFF | ON | ON | 400 |
| OFF | ON | OFF | 6400 |
| OFF | OFF | ON | 1600 |
| OFF | OFF | OFF | 25600 |

`gear_ratio` is motor revolutions per output-shaft revolution. Leave it at
`1.0` for direct drive.

```bash
roslaunch stepper_gpio_driver driver.launch \
  pulses_per_revolution:=200 gear_ratio:=1.0
```

## ROS interface

Send angle, output-shaft RPM, and direction as one atomic command:

```bash
rostopic pub -1 /stepper_gpio/command stepper_gpio_driver/MotorCommand \
  "{angle_deg: 90.0, speed_rpm: 30.0, direction: 1}"
```

- `/stepper_gpio/command` (`stepper_gpio_driver/MotorCommand`): positive angle
  in degrees, positive output-shaft speed in RPM, and direction `+1` or `-1`.
- `/stepper_gpio/stop` (`std_msgs/Empty`): requests a stop and forces the pulse
  output low at the next pulse edge.
- `/stepper_gpio/status` (`stepper_gpio_driver/MotorStatus`): latched status with
  `idle`, `queued`, `running`, `completed`, `stopping`, `stopped`, `rejected`,
  or `error` state.
- `/stepper_gpio/run_test` (`std_srvs/Trigger`): queues a small pulse test only
  when `test_mode` is enabled.

Stop a motion:

```bash
rostopic pub -1 /stepper_gpio/stop std_msgs/Empty "{}"
```

The direction polarity depends on motor phase wiring. Use
`invert_direction:=true` if `+1` moves in the opposite physical direction.

## Point test

The test launch disables normal commands, waits two seconds, then emits only
five pulses at 100 Hz. With 200 pulses/revolution this is 9 degrees; finer
subdivision settings move less.

```bash
roslaunch stepper_gpio_driver test.launch
```

For a software-only check that does not touch GPIO:

```bash
roslaunch stepper_gpio_driver test.launch dry_run:=true
```

After an automatic test, another identical test can be requested with:

```bash
rosservice call /stepper_gpio/run_test
```

The `completed` status proves that all GPIO writes completed. It cannot prove
mechanical rotation because these two GPIO outputs provide no motor feedback.

## Timing limit

The GEACX1 exposes these pins through a sysfs GPIO driver, not hardware PWM.
Pulse generation therefore has normal Linux scheduling jitter, especially near
20 kHz. Use an oscilloscope to validate high-speed operation. For hard
real-time or tightly controlled pulse trains, use a hardware timer, PWM, or an
external motion controller.
