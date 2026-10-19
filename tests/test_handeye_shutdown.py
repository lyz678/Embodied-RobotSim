"""Exercise actual subprocess groups without starting ROS or Isaac."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

path = Path(__file__).resolve().parents[1]/'src/manipulation/handeye_demo/launch.py'
spec = importlib.util.spec_from_file_location('handeye_launcher', path)
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)

STUBBORN = "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); print('ready',flush=True); time.sleep(60)"


class ShutdownTests(unittest.TestCase):
    def test_stubborn_child_is_killed_within_deadline(self):
        p = subprocess.Popen([sys.executable, '-c', STUBBORN], start_new_session=True, stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(p.stdout.readline().strip(), 'ready')
            start = time.monotonic()
            launcher.stop_processes([p], grace_seconds=.15)
            self.assertLess(time.monotonic()-start, 2.)
            self.assertEqual(p.wait(timeout=1), -signal.SIGKILL)
        finally:
            launcher.stop_processes([p], grace_seconds=.05)
            p.stdout.close()

    def test_exited_group_leader_does_not_leave_descendant_alive(self):
        code = "import subprocess,sys; p=subprocess.Popen([sys.executable,'-c'," + repr(STUBBORN) + "]); print(p.pid,flush=True)"
        p = subprocess.Popen([sys.executable, '-c', code], start_new_session=True, stdout=subprocess.PIPE, text=True)
        child_pid = int(p.stdout.readline())
        try:
            self.assertEqual(p.stdout.readline().strip(), 'ready')
            self.assertEqual(p.wait(timeout=2), 0)
            launcher.stop_processes([p], grace_seconds=.15)
            # An orphan may briefly remain as a zombie, but must no longer execute.
            end = time.monotonic()+1
            while time.monotonic() < end:
                status = Path(f'/proc/{child_pid}/status')
                if not status.exists() or '\nState:\tZ' in status.read_text():
                    break
                time.sleep(.02)
            else:
                self.fail('descendant survived shutdown after its leader exited')
        finally:
            launcher.stop_processes([p], grace_seconds=.05)
            p.stdout.close()


    def test_repeated_ctrl_c_finishes_launcher_cleanup(self):
        # Replace expensive services with real stubborn processes; exercise main's signal handlers.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = path.with_name('config.yaml')
            harness = f"""
import importlib.util,sys,os
from pathlib import Path
spec=importlib.util.spec_from_file_location('launcher',{str(path)!r})
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
m.subprocess.check_output=lambda *a,**k:'/tmp'
m.subprocess.run=lambda *a,**k:None
real_popen=m.subprocess.Popen
def spawn(command,**kwargs):
    p=real_popen([sys.executable,'-c',{STUBBORN!r}],**kwargs)
    with open({str(root/'pids')!r},'a') as f:f.write(str(p.pid)+'\\n')
    return p
m.subprocess.Popen=spawn
os.environ['ISAAC_SIM_PYTHON']=sys.executable
sys.argv=['launch.py','all','--output',{str(root/'output')!r},'--config',{str(config)!r},'--domain-id',str(os.getpid()+1000)]
raise SystemExit(m.main())
"""
            p = subprocess.Popen([sys.executable, '-c', harness], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            children = []
            try:
                deadline = time.monotonic()+5
                while time.monotonic() < deadline:
                    pidfile = root/'pids'
                    children = pidfile.read_text().splitlines() if pidfile.exists() else []
                    if len(children) == 6:
                        break
                    if p.poll() is not None:
                        self.fail(p.stdout.read())
                    time.sleep(.02)
                self.assertEqual(len(children), 6)
                time.sleep(.1)
                p.send_signal(signal.SIGINT)
                time.sleep(.1)
                p.send_signal(signal.SIGINT)
                output, _ = p.communicate(timeout=8)
                self.assertEqual(p.returncode, 130, output)
                for pid in children:
                    status = Path(f'/proc/{pid}/status')
                    self.assertTrue(not status.exists() or '\nState:\tZ' in status.read_text(), pid)
            finally:
                if p.poll() is None:
                    p.kill();p.wait()
                for pid in children:
                    try:os.killpg(int(pid), signal.SIGKILL)
                    except ProcessLookupError:pass
                p.stdout.close()


if __name__ == '__main__':
    unittest.main()
