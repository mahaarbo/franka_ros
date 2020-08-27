# ROS integration for Franka Emika research robots

[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## ROS 2
`libfranka` must be installed locally and you must use the correct version. Use the cmake cache entry `-DFranka_DIR:PATH=/path/to/libfranka/build`.

As xacro integration in ros2 is not complete for python-based launch scripts, urdfs are included for Franka Emika Panda with and without gripper. 

For ROS 2 Eloquent and Dashing, there are URDFs included instead of using the xacro. For Foxy and above, the `command` method can be used to generate the URDFs from the xacro. 

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros
