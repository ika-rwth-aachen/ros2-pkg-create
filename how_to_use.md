This file is a work in progress to showcase how to use the ROS 2 controller package generator.

## Generate controller package

```bash
ros2-pkg-create ~/my_ws/src/ --template ros2_controller_pkg --use-local-templates --defaults --package-name custom_franka_controller --controller-name custom_controller
```

## Build new package

```bash
colcon build --symlink-install --packages-up-to custom_franka_controller --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Set new joint command as reference

Command to publish message which moves robot to same pose as `ros2 launch franka_bringup move_to_start_example_controller.launch.py`:

**WARNING: Make sure the robot is very close to this pose already, or it will get a high power input and jerk violently before being stopped because it exceeded joint velocity limits.**

You can adjust the joint angles slightly (difference <0.05) to make the robot move slightly.

```bash
ros2 topic pub /custom_controller/cmd_vel custom_franka_controller_interfaces/msg/JointPositions "{ data: [0.0, -0.785, 0.0, -2.355, 0.0, 1.571, 0.785] }"
```

## See published command state

```bash
ros2 topic echo /custom_controller/commanded_state
```
