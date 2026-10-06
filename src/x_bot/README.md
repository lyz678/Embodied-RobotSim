# x_bot

ROS 2 Jazzy robot package for Embodied-RobotSim's Isaac Sim 6.1 backend.
The robot combines a differential-drive base, Franka FR3, a simulated MID-360
and stereo RGB-D cameras.

Use the workspace's [setup and simulation guide](../../README.md).
Run root `start_*.sh` entry points rather than legacy Gazebo launch files.

| Directory | Purpose |
|---|---|
| `isaac_sim/` | Robot import, scene construction, physics, sensors and ROS integration |
| `urdf/`, `meshes/` | Shared robot description and meshes |
| `config/` | Isaac controllers, MoveIt, mapping and manipulation task settings |
| `launch/` | ROS controllers, MoveIt and mapping launches |
| `scripts/` | Grasp tasks, gripper actions and physical verification |
| `rviz/octomap.rviz` | Single RViz configuration for clouds, maps, trajectories and navigation |
| `worlds/` | Original scene definitions used by the asset migration pipeline |

Common runtime is in `../../scripts/robot_services.sh`; Isaac runtime environment
is in `../../scripts/isaac_runtime.sh`. Scene assets are prepared by
`../../scripts/assets/migrate_gazebo_scenes.py`. FAST-LIO and Nav2 settings belong
to `../x_bot_localization/config`, outside this package.

Do not remove physical verification, telemetry or fixture collision code merely
because it contains debugging/validation names: the manipulation demo uses these
modules for its success criteria and collision planning.
