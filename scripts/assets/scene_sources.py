"""Pinned source scenes shared by Gazebo and Isaac asset preparation."""

from pathlib import Path
import shutil
import subprocess
import tarfile

REPOSITORY = Path(__file__).resolve().parents[2]
SOURCE_REVISION = "92b0409ccf83549e74d03966bdde7f0f700ff927"


def extract_sources(destination, revision):
    source = destination / "source"
    source.mkdir(parents=True, exist_ok=True)
    commit = subprocess.check_output(
        ["git", "rev-parse", revision], cwd=REPOSITORY, text=True
    ).strip()
    if subprocess.run(
        ["git", "cat-file", "-e", commit + ":src/x_bot/models"],
        cwd=REPOSITORY,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode:
        raise RuntimeError(
            "Pinned Gazebo assets unavailable in Git history. Fetch full main history (git fetch --unshallow origin, for a shallow clone)."
        )
    marker = source / "revision.txt"
    if marker.exists() and marker.read_text().strip() == commit:
        return source, commit
    archive = subprocess.Popen(
        ["git", "archive", commit, "src/x_bot/models", "src/x_bot/worlds"],
        cwd=REPOSITORY,
        stdout=subprocess.PIPE,
    )
    with tarfile.open(fileobj=archive.stdout, mode="r|") as tar:
        for member in tar:
            if member.isfile():
                relative = Path(member.name).relative_to("src/x_bot")
                target = source / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(tar.extractfile(member).read())
    if archive.wait() != 0:
        raise RuntimeError("git archive failed")
    # Google scanned OBJ materials refer to texture basenames even though
    # Gazebo finds them in materials/textures. Supply that search path to Kit.
    for mtl in source.rglob("*.mtl"):
        for line in mtl.read_text(errors="replace").splitlines():
            if line.startswith(("map_Kd ", "map_Ks ", "map_Bump ", "bump ")):
                name = line.split(maxsplit=1)[1].strip()
                target = mtl.parent / name
                if not target.exists():
                    matches = list(mtl.parent.parent.rglob(Path(name).name))
                    if len(matches) != 1:
                        raise FileNotFoundError(f"Material texture {name}: {mtl}")
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(matches[0], target)
    marker.write_text(commit + "\n")
    return source, commit
