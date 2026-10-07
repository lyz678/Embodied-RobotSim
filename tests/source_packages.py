"""Import moved numerical packages from this checkout for offline regression."""
import os
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
for group in ('robot','control','mapping','localization','simulation'):
    for package in (ROOT/'src'/group).iterdir():
        if package.is_dir(): sys.path.insert(0, str(package))
os.environ.setdefault('X_BOT_BASE_MOTION_CONFIG', str(ROOT/'src/control/x_bot_control/config/base_motion.yaml'))
