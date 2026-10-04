#!/usr/bin/env python3
"""Close shells belonging to robot service terminals, leaving other terminals alone."""

import os
import signal
from pathlib import Path


def ancestors():
    result = set()
    pid = os.getpid()
    while pid and pid not in result:
        result.add(pid)
        try:
            status = (Path('/proc') / str(pid) / 'status').read_text()
            pid = int(next(line.split()[1] for line in status.splitlines() if line.startswith('PPid:')))
        except (OSError, StopIteration, ValueError):
            break
    return result


def close_terminals():
    protected = ancestors()
    closed = 0
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name) in protected:
            continue
        try:
            if process.stat().st_uid != os.getuid():
                continue
            if (process / 'comm').read_text().strip() not in {'bash', 'zsh', 'sh'}:
                continue
            environment = (process / 'environ').read_bytes().split(b'\0')
            if b'ROBOT_SIM_TERMINAL=Embodied-RobotSim' not in environment:
                continue
            # Only close terminal shells, not a marked background script.
            if not os.readlink(process / 'fd/0').startswith('/dev/pts/'):
                continue
            os.kill(int(process.name), signal.SIGHUP)
            closed += 1
        except (OSError, ProcessLookupError):
            continue
    return closed


if __name__ == '__main__':
    print(f'已关闭 {close_terminals()} 个机器人服务终端 shell。')
