evart_bridge
============

evart_bridge wraps the EVaRT(tm) client into a ROS package. EVaRT(tm)
client can be used to retrieve motion capture data from Cortex, a
motion capture software from Motion Analysis.

This package is composed of one node called 'evart' which publishes
transformation w.r.t the motion capture world frame.

* [Project webpage on ros.org: tutorial and API reference] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]


Setup
-----

This package can be compiled like any other ROS package using `rosdep`
and `rosmake`.

For more information, refer to the [ROS tutorial]
[ros-tutorial-building-pkg].


Documentation
-------------

The documentation is available on the project [ROS homepage]
[ros-homepage].


[github-homepage]: https://github.com/laas/evart_bridge
[ros-homepage]: http://www.ros.org/wiki/evart_bridge
[ros-tutorial-building-pkg]: http://www.ros.org/wiki/ROS/Tutorials/BuildingPackages "Building a ROS Package"
