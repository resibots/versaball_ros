# Versaball control

**User the Versaball on our Omnigrasper robot.**

## Technical details

In this setup, the versaball is (slightly) inflated or deflated with two pumps and two electro-valves. These four devices are in turn switched on and off thankts to a USB-controlled rellays board (Phidgets InterfaceKit 0/0/4).

## Dependencies

Before installing this ROS node, please get [`phidget_drivers`](https://github.com/ros-drivers/phidgets_drivers) from ros-drivers. The node `phidgets_ik` from that package must be run before `versaball`.
