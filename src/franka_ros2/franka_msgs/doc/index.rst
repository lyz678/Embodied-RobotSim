franka_msgs
===========

This package contains the definitions for the different gripper actions and robot state message.

Message Types
-------------

This package provides ROS2 message, service, and action definitions specific to Franka robots:

**Action Definitions:**
  - Gripper control actions (homing, move, grasp)
  - Error recovery actions

**Service Definitions:**
  - Robot parameter setting services
  - Collision behavior configuration
  - Stiffness and load configuration

**Message Definitions:**
  - Robot state messages
  - Gripper state and feedback messages

Usage
-----

These message definitions are used throughout the franka_ros2 ecosystem:

- :doc:`franka_gripper <../../franka_gripper/doc/index>` uses the gripper action definitions
- :doc:`franka_robot_state_broadcaster <../../franka_robot_state_broadcaster/doc/index>` publishes robot state messages
- :doc:`franka_bringup <../../franka_bringup/doc/index>` uses service definitions for robot parameter setting

The message definitions ensure consistent communication interfaces across all Franka ROS2 packages.