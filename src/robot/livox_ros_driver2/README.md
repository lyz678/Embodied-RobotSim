# Simulation-only Livox interfaces

This package intentionally exports the ROS package name `livox_ros_driver2` so
FAST-LIO can consume the upstream `CustomMsg` / `CustomPoint` wire contract.
Only message definitions are included, from Livox-SDK/livox_ros_driver2 commit
`21445540f0d100dc86a7e6df312dd70bbdb4afdf` (MIT); comments were shortened.
No Livox SDK, device discovery or hardware driver is included/required.

Do **not** build this alongside a full package of the same name. For real
hardware, exclude this directory with COLCON_IGNORE and use the full upstream
driver instead. Isaac's embedded Python publishes standard sensor_msgs only.
