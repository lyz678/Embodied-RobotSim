#!/usr/bin/env python3
"""Run only this experiment's services in an isolated ROS domain; own process cleanup."""
import argparse
import fcntl
import json
import tempfile
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time


def stop_processes(processes, grace_seconds=2.0):
    """Stop owned sessions, including descendants whose group leader already exited."""
    groups = {p.pid for p in processes}  # every child was started with start_new_session
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for group in list(groups):
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                groups.discard(group)
        deadline = time.monotonic() + grace_seconds
        while groups and time.monotonic() < deadline:
            for p in processes:
                p.poll()  # reap direct children before checking their process groups
            for group in list(groups):
                try:
                    os.killpg(group, 0)
                except ProcessLookupError:
                    groups.discard(group)
            if groups:
                time.sleep(.05)
        if not groups:
            break
    for p in processes:
        p.poll()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['all','collect','grasp'])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('config.yaml'))
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--domain-id',type=int,default=73)
    args=parser.parse_args()
    if args.resume and args.command!='grasp':
        parser.error('--resume is only supported by grasp')
    root=Path(__file__).resolve().parents[3]
    args.output=args.output.resolve(); args.output.mkdir(parents=True,exist_ok=True)
    args.config=args.config.resolve()
    if args.command=='grasp':
        args.config=args.output/'config.yaml'
    if not args.config.is_file():
        raise SystemExit(f'Config not found: {args.config}')
    if args.command in ('all','collect') and (args.output/'grid/samples.json').exists():
        raise SystemExit('Output contains observations; use a fresh directory or an offline command.')
    if args.command=='grasp':
        if (args.output/'grasps.json').exists() and not args.resume:
            raise SystemExit('Output contains grasp results; preserve them and choose a fresh experiment.')
        calibration=args.output/'calibration.json'
        if not calibration.exists() or not json.loads(calibration.read_text())['complete']:
            raise SystemExit('A complete calibration is required before starting grasp services.')
    if args.command in ('all','collect'):
        frozen=args.output/'config.yaml'
        if args.config!=frozen:
            shutil.copyfile(args.config,frozen)
        args.config=frozen
    domain_lock=open(Path(tempfile.gettempdir())/f'handeye-demo-domain-{args.domain_id}.lock','a')
    try:
        fcntl.flock(domain_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit(f'Another handeye experiment owns ROS domain {args.domain_id}; choose another --domain-id.')
    environment=dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id), RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    def share(package):
        return subprocess.check_output(['ros2','pkg','prefix','--share',package],env=environment,text=True).strip()
    # Validate paths before starting services; never stop the user's existing sessions.
    shares={p:share(p) for p in ('x_bot','franka_description','x_bot_control','x_bot_isaac','x_bot_moveit_config','x_bot_manipulation')}
    isaac=Path(os.environ.get('ISAAC_SIM_PYTHON',str(Path(os.environ.get('ISAAC_SIM_PATH',str(Path.home()/'isaacsim')))/'python.sh')))
    if not isaac.is_file():
        raise SystemExit(f'Isaac python.sh not found: {isaac}')
    subprocess.run([sys.executable,str(Path(__file__).with_name('run.py')),'assets','--output',str(args.output)],check=True)
    logs=args.output/'logs'/f'{args.command}-{time.time_ns()}'; logs.mkdir(parents=True,exist_ok=True)
    print(f'Experiment service logs: {logs}', flush=True)
    commands={
        'controllers':['ros2','launch','x_bot_isaac','controllers.launch.py'],
        'moveit':['ros2','launch','x_bot_moveit_config','move_group.launch.py','sim_backend:=isaac','use_sim_time:=true'],
        'odom':['ros2','run','tf2_ros','static_transform_publisher','--frame-id','odom','--child-frame-id','base_footprint'],
        'arm':['ros2','run','x_bot_manipulation','robot_actions','--ros-args','-p','use_sim_time:=true'],
        'isaac':[str(isaac),str(root/'src/simulation/x_bot_isaac/runtime/run_sim.py'),
                 '--world','empty','--package-share',shares['x_bot'],'--franka-share',shares['franka_description'],
                 '--controller-config',str(Path(shares['x_bot_control'])/'config/isaac_controllers.yaml'),
                 '--handeye-config',str(args.config),'--handeye-assets',str(args.output/'assets')],
    }
    if args.headless:
        commands['isaac'].append('--headless')
    ros_lib=isaac.resolve().parent/'exts/isaacsim.ros2.core/jazzy/lib'
    environment['LD_LIBRARY_PATH']=environment.get('LD_LIBRARY_PATH','')+':'+str(ros_lib)
    processes=[]; handles=[]; runner=None
    stop_signal = signal.SIGINT
    stop_requested = False
    def request_stop(signum, frame):
        nonlocal stop_signal, stop_requested
        stop_signal = signum
        stop_requested = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        for name,command in commands.items():
            if stop_requested:
                return 128 + stop_signal
            handle=(logs/f'{name}.log').open('w'); handles.append(handle)
            processes.append((name,subprocess.Popen(command,env=environment,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)))
        if stop_requested:
            return 128 + stop_signal
        runner_command=[sys.executable,str(Path(__file__).with_name('run.py')),args.command,
                        '--config',str(args.config),'--output',str(args.output)]
        if args.resume:
            runner_command.append('--resume')
        runner=subprocess.Popen(runner_command,env=environment,start_new_session=True)
        while runner.poll() is None:
            if stop_requested:
                return 128 + stop_signal
            for name,p in processes:
                if p.poll() is not None:
                    raise RuntimeError(f'{name} exited with {p.returncode}; inspect {logs/name}.log')
            time.sleep(.5)
        return runner.returncode
    except KeyboardInterrupt:
        print('正在关闭手眼标定实验及其子进程…', flush=True)
        return 128 + stop_signal
    finally:
        if stop_requested:
            print('正在关闭手眼标定实验及其子进程…', flush=True)
        # Repeated Ctrl+C must not interrupt cleanup and leave isolated sessions behind.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop_processes([p for _, p in processes] + ([runner] if runner else []))
        for handle in handles:
            handle.close()



if __name__=='__main__':
    raise SystemExit(main())
