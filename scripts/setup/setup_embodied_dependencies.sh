#!/usr/bin/env bash
# Explicit optional installation, never run automatically by a simulation entry.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [[ "${1:-}" == --local ]]; then
    [[ "$ROS_DISTRO" == jazzy ]] || { echo "Local overlay currently supports Jazzy" >&2; exit 2; }
    python3 - "$ROOT_DIR/.cache/embodied_sysroot" <<'PYTHON'
from pathlib import Path
import subprocess,re,os
import sys
root=Path(sys.argv[1]);debs=root/'debs';debs.mkdir(parents=True,exist_ok=True)
queue=['ros-jazzy-rosbridge-server','ros-jazzy-web-video-server'];seen=set();missing=[];env=dict(os.environ,LC_ALL='C')
while queue:
 p=queue.pop()
 if p in seen:continue
 seen.add(p)
 status=subprocess.run(['dpkg-query','-W','-f=${db:Status-Status}',p],capture_output=True,text=True)
 if status.stdout=='installed':continue
 missing.append(p)
 data=subprocess.check_output(['apt-cache','depends',p],env=env,text=True)
 queue += re.findall(r'^\s*(?:Depends|PreDepends): ([a-zA-Z0-9][a-zA-Z0-9.+-]*)$',data,re.M)
print('Preparing optional Web packages in user cache:',len(missing),flush=True)
download=[p for p in missing if not list(debs.glob(p+'_*.deb'))]
if download: subprocess.run(['apt-get','download',*download],cwd=debs,env=env,check=True,stdout=subprocess.DEVNULL)
for p in debs.glob('*.deb'):subprocess.run(['dpkg-deb','-x',str(p),str(root)],check=True)
(root/'packages.txt').write_text('\n'.join(missing)+'\n')
print('Local Web dependency overlay ready:',root)
PYTHON
    cat > "$ROOT_DIR/.cache/embodied_sysroot/env.bash" <<'ENV'
EMBODIED_DEP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMBODIED_ROS_PREFIX="$EMBODIED_DEP_ROOT/opt/ros/jazzy"
export AMENT_PREFIX_PATH="$EMBODIED_ROS_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PYTHONPATH="$EMBODIED_ROS_PREFIX/lib/python3.12/site-packages:$EMBODIED_DEP_ROOT/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$EMBODIED_ROS_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ENV
elif [[ $# == 0 ]]; then
    sudo apt-get update
    sudo apt-get install -y "ros-$ROS_DISTRO-rosbridge-server" "ros-$ROS_DISTRO-web-video-server" python3-venv
else
    echo "Usage: $0 [--local]" >&2; exit 2
fi
python3 -m venv --system-site-packages "$ROOT_DIR/.venv/agent"
"$ROOT_DIR/.venv/agent/bin/python" -m pip install openai fastapi uvicorn websockets
printf 'Agent Python: %s\nSet DASHSCOPE_API_KEY before starting start_embodied.sh.\n' "$ROOT_DIR/.venv/agent/bin/python"
