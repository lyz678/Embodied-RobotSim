#!/usr/bin/env python3
"""Standalone Isaac Sim 6.1 entry point for Embodied-RobotSim."""

from __future__ import annotations

import argparse
import sys
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
    parser.add_argument("--publish-odom-tf", action="store_true")
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
    from ros_bridge import create_camera_graphs, create_control_graph, create_lidar
    from scene_builder import build_scene

    temp_dir = None
    try:
        for extension in (
            "isaacsim.asset.importer.urdf",
            "isaacsim.robot.schema",
            "omni.scene.optimizer.core",
            "isaacsim.ros2.core",
            "isaacsim.ros2.bridge",
            "isaacsim.ros2.control",
            "isaacsim.robot.wheeled_robots.nodes",
            "isaacsim.sensors.experimental.rtx",
        ):
            app_utils.enable_extension(extension)
        simulation_app.update()

        stage = stage_utils.create_new_stage()
        build_scene(stage, args.world, args.package_share.resolve())

        urdf_path, temp_dir = generate_urdf(args.package_share.resolve())
        robot_usd = import_robot_asset(
            urdf_path,
            temp_dir,
            {
                "x_bot": args.package_share.resolve(),
                "franka_description": args.franka_share.resolve(),
            },
        )
        add_robot_reference(stage, robot_usd, args.x, args.y, args.yaw)
        simulation_app.update()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            simulation_app.update()

        articulation = find_articulation_root(stage)
        articulation_path = str(articulation.GetPath())
        configure_joint_drives(stage)
        create_control_graph(stage, articulation_path, args.controller_config.resolve(), args.publish_odom_tf)
        create_camera_graphs(stage)
        lidar_sensor = create_lidar(stage)
        # Keep the sensor wrapper referenced for the lifetime of its render product.
        _ = lidar_sensor

        SimulationManager.setup_simulation(dt=1.0 / 120.0, device="cpu")
        if not args.headless:
            ViewportManager.set_camera_view(
                "/OmniverseKit_Persp", eye=[3.5, 3.5, 2.8], target=[0.0, 0.0, 0.8]
            )
        app_utils.play()
        for _ in range(4):
            simulation_app.update()
        set_initial_joint_state(articulation_path)

        carb.log_info(
            f"Embodied-RobotSim Isaac backend ready: world={args.world}, robot={ROBOT_PRIM_PATH}, "
            "cmd_vel=/x_bot/cmd_vel"
        )
        while simulation_app.is_running():
            simulation_app.update()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        carb.log_error(f"Embodied-RobotSim Isaac backend failed: {error}")
        return_code = 1
    else:
        return_code = 0
    finally:
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
