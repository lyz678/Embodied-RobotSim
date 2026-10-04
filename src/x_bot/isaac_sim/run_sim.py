#!/usr/bin/env python3
"""Standalone Isaac Sim 6.1 entry point for Embodied-RobotSim."""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

from isaacsim import SimulationApp


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", choices=("simple_room", "manipulation_test"), default="simple_room")
    parser.add_argument("--package-share", type=Path, required=True)
    parser.add_argument("--franka-share", type=Path, required=True)
    parser.add_argument("--controller-config", type=Path, required=True)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--mid360-xyz", default='0.25 0 0.0875')
    parser.add_argument("--mid360-rpy", default='0 0 0')
    parser.add_argument("--headless", action="store_true")
    args, _ = parser.parse_known_args()
    return args


def main() -> int:
    args = _arguments()
    simulation_app = SimulationApp(
        {
            "headless": args.headless,
            "renderer": "RayTracedLighting",
            "anti_aliasing": 2,
        }
    )

    import carb
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    import omni.usd
    from isaacsim.core.rendering_manager import ViewportManager
    from isaacsim.core.simulation_manager import SimulationManager

    module_dir = Path(__file__).resolve().parent
    sys.path.insert(0, str(module_dir))
    from robot_importer import (
        ROBOT_PRIM_PATH,
        add_robot_reference,
        cleanup_temp_dir,
        configure_joint_drives,
        find_articulation_root,
        generate_urdf,
        import_robot_asset,
        set_initial_joint_state,
    )
    from ros_bridge import create_camera_graphs, create_control_graph
    from scene_builder import build_scene

    temp_dir = None
    lidar_sensor = None
    return_code = 0
    try:
        for extension in (
            "isaacsim.asset.importer.urdf",
            "isaacsim.robot.schema",
            "omni.scene.optimizer.core",
            "isaacsim.ros2.core",
            "isaacsim.ros2.bridge",
            "isaacsim.ros2.control",
            "isaacsim.robot.wheeled_robots.nodes",
            "isaacsim.sensors.experimental.physics",
        ):
            app_utils.enable_extension(extension)
        simulation_app.update()

        urdf_path, temp_dir = generate_urdf(args.package_share.resolve(), args.mid360_xyz, args.mid360_rpy)
        robot_usd = import_robot_asset(
            urdf_path,
            temp_dir,
            {
                "x_bot": args.package_share.resolve(),
                "franka_description": args.franka_share.resolve(),
            },
        )
        # The URDF importer opens its conversion stages in the global USD
        # context. Build the simulation stage after conversion so rendering,
        # physics and the robot reference all use the same active stage.
        stage = stage_utils.create_new_stage()
        build_scene(stage, args.world, args.package_share.resolve())
        add_robot_reference(stage, robot_usd, args.x, args.y, args.yaw)
        simulation_app.update()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            simulation_app.update()

        articulation = find_articulation_root(stage)
        articulation_path = str(articulation.GetPath())
        configure_joint_drives(stage)
        create_control_graph(stage, articulation_path, args.controller_config.resolve(), urdf_path)
        create_camera_graphs(stage)
        SimulationManager.setup_simulation(dt=1.0 / 200.0, device="cpu")
        from mid360 import Mid360
        lidar_sensor = Mid360(stage)
        # Initialize tensor views without advancing the timeline, so the arm
        # and base start at rest before the first published physics sample.
        SimulationManager.initialize_physics()
        set_initial_joint_state(articulation_path)
        if not args.headless:
            ViewportManager.set_camera_view(
                "/OmniverseKit_Persp", eye=[3.5, 3.5, 2.8], target=[0.0, 0.0, 0.8]
            )
        app_utils.play()

        carb.log_info(
            f"Embodied-RobotSim Isaac backend ready: world={args.world}, robot={ROBOT_PRIM_PATH}, "
            "cmd_vel=/x_bot/cmd_vel"
        )
        while simulation_app.is_running():
            lidar_sensor.update()
            simulation_app.update()
            if lidar_sensor.error:
                raise lidar_sensor.error
    except KeyboardInterrupt:
        pass
    except Exception as error:
        carb.log_error(f"Embodied-RobotSim Isaac backend failed: {error}\n{traceback.format_exc()}")
        return_code = 1
    else:
        return_code = 0
    finally:
        if lidar_sensor is not None:
            lidar_sensor.close()
        try:
            app_utils.stop()
        except Exception:
            pass
        simulation_app.close()
        if temp_dir is not None:
            cleanup_temp_dir(temp_dir)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
