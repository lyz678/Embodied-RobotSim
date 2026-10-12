#!/usr/bin/env python3
"""Download verified Q4 weights and build a pinned llama.cpp CUDA server."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
REVISION = '5e1d74043eb84f5f82471a1c5d4c8171a4c0fcba'
MODEL_REVISION = 'e87f176479d0855a907a41277aca2f8ee7a09523'
FILES = {
    'Qwen3.5-4B-Q4_K_M.gguf': '00fe7986ff5f6b463e62455821146049db6f9313603938a70800d1fb69ef11a4',
    'mmproj-F16.gguf': 'cd88edcf8d031894960bb0c9c5b9b7e1fea6ebee02b9f7ce925a00d12891f864',
}


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', type=Path, default=Path(os.getenv('QWEN_HOME', ROOT/'.cache/qwen')))
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--cuda-arch', default='120', help='RTX 5060 Ti: 120; override for other GPUs')
    args = parser.parse_args()
    home = args.home.resolve()
    for command in ('git', 'cmake', 'curl', 'nvidia-smi'):
        if not shutil.which(command):
            parser.error(f'Missing {command}; install build-essential cmake git curl and NVIDIA/CUDA first')
    nvcc = shutil.which('nvcc') or '/usr/local/cuda/bin/nvcc'
    if not Path(nvcc).is_file():
        parser.error('CUDA nvcc not found')
    source, models = home/'llama.cpp', home/'models'
    models.mkdir(parents=True, exist_ok=True)
    for name, expected in FILES.items():
        target = models/name
        if not target.exists() or sha256(target) != expected:
            part = target.with_suffix('.gguf.part')
            url = f'https://huggingface.co/unsloth/Qwen3.5-4B-GGUF/resolve/{MODEL_REVISION}/{name}'
            subprocess.run(['curl', '--fail', '--location', '--retry', '3', '-C', '-', url, '-o', str(part)], check=True)
            if sha256(part) != expected:
                raise RuntimeError(f'SHA256 mismatch: {part}; remove the partial file before retrying')
            part.replace(target)
        print(f'Verified {name}', flush=True)
    if not source.exists():
        subprocess.run(['git', 'init', str(source)], check=True)
        subprocess.run(['git', '-C', str(source), 'remote', 'add', 'origin', 'https://github.com/ggml-org/llama.cpp.git'], check=True)
    head = subprocess.run(['git', '-C', str(source), 'rev-parse', '--verify', 'HEAD'],
                          capture_output=True, text=True)
    current = head.stdout.strip() if head.returncode == 0 else ''
    if current != REVISION:
        if subprocess.check_output(['git', '-C', str(source), 'status', '--porcelain'], text=True).strip():
            raise RuntimeError(f'Local changes in {source}; refusing to overwrite')
        subprocess.run(['git', '-C', str(source), 'fetch', '--depth', '1', 'origin', REVISION], check=True)
        subprocess.run(['git', '-C', str(source), 'checkout', '--detach', REVISION], check=True)
    subprocess.run(['cmake', '-S', str(source), '-B', str(source/'build'), '-DGGML_CUDA=ON',
                    f'-DCMAKE_CUDA_COMPILER={nvcc}', f'-DCMAKE_CUDA_ARCHITECTURES={args.cuda_arch}',
                    '-DCMAKE_BUILD_TYPE=Release', '-DLLAMA_BUILD_TESTS=OFF', '-DLLAMA_BUILD_EXAMPLES=OFF'], check=True)
    subprocess.run(['cmake', '--build', str(source/'build'), '--target', 'llama-server', '-j', str(args.jobs)], check=True)
    (home/'installed.json').write_text(json.dumps(dict(llama_revision=REVISION, model_revision=MODEL_REVISION,
                                                    sha256=FILES), indent=2)+'\n')
    print(f'Ready: QWEN_HOME={home} bash src/embodied/llm_agent/start_local_model.sh')


if __name__ == '__main__':
    main()
