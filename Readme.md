# Versaball control

Control the Versaball for our Omnigrasper robot.

## Usage

Simply run `roslaunch versaball versaball.launch` after compiling. You then have threee services at hand:

- `/versaball/prepare_grasp`: call this to get the versaball ready for grasping
- `/versaball/grasp`: when called, air is sucked out of the gripper, to grasp an object
- `/versaball/release`: release it!

## Technical details

In this setup, the Versaball is (slightly) inflated or deflated with two pumps and two electro-valves. These four devices are in turn switched on and off thankts to a USB-controlled rellays board (Phidgets InterfaceKit 0/0/4). We use a basic time-based control scheme for the Versaball.

To change how much the Versaball is inflated or deflated, use dynamic_reconfigure.

## Dependencies

Before installing this ROS node, please get [`phidget_drivers`](https://github.com/ros-drivers/phidgets_drivers) from ros-drivers. The node `phidgets_ik` from that package must be run before `Versaball`.
