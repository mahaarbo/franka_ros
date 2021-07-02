# ROS integration for Franka Emika research robots

[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## ROS 2
`libfranka` must be installed locally and you must use the correct version. Use the cmake cache entry `-DFranka_DIR:PATH=/path/to/libfranka/build` when running colcon build.

To run the franka visualization:

1. build and source with [`ros2_control`][ros2control]
2. Test the visualizer with `ros2 launch franka_visualization franka_visualization.launch.py robot_ip:=<fci-ip>`
3. Test the hardware interface with `ros2 launch franka_hw franka_hw_test.launch.py robot_ip:=<fci-ip>` 
    1. Then run `ros2 control load_controller --set-state start forward_command_controller_velocity`
    2. Publish some joint velocity commands to test it out
    3. Switch to `forward_command_controller_position` and be ready on the emergency stop, because that one does not work quite yet.

4. Test the `franka_gripper` action server (Did not work yet)

Note: This is still in very early developments, and the packages are cluttered with both the new and the old implementations. To see which files are in use, check whether there is a `COLCON_IGNORE` file in the folder, and if there isn't, check the `CMakeLists.txt` to see which files are actually used. Basic control of the robot in joint velocity and joint torque is possible, but there are still things that must be further developed in `ros2_control` and in this package before joint position, and the cartesian interface can be used.
 
## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros
[ros2control]:https://github.com/ros-controls/ros2_control/
