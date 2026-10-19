"""AprilGrid + Daniilidis solving and independent truth-based scoring."""
import csv
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from core import METHOD, errors, solve, stats


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)


def grasp_history(directory, config, calibration, resume=False):
    """Resume only missing trial IDs under identical frozen inputs."""
    directory = Path(directory)
    path, manifest_path = directory/'grasps.json', directory/'grasp_manifest.json'
    manifest = {'config': config, 'calibration': calibration}
    if path.exists() and not resume:
        raise RuntimeError('grasp results already exist; use --resume to finish the frozen experiment')
    if resume:
        if not manifest_path.exists() or json.loads(manifest_path.read_text()) != manifest:
            raise RuntimeError('resume requires the identical frozen config and calibration manifest')
    else:
        write_json(manifest_path, manifest)
    trials = json.loads(path.read_text()) if path.exists() else []
    ids = [t['trial'] for t in trials]
    if len(set(ids)) != len(ids) or not set(ids) <= set(range(len(config['grasp']['trials']))):
        raise RuntimeError('duplicate or invalid stored grasp trial identifiers')
    return trials


def calibrate(directory, config):
    directory = Path(directory)
    for name in ('evaluation.json', 'evaluation_samples.csv', 'errors.png'):
        (directory/name).unlink(missing_ok=True)
    path = directory/'grid/samples.json'
    samples = json.loads(path.read_text()) if path.exists() else []
    counts = {split: sum(s['split'] == split for s in samples) for split in ('train', 'test')}
    result = {'complete': False, 'method': METHOD, 'counts': counts}
    try:
        if any(counts[k] < config['sampling'][k] for k in counts):
            raise ValueError('insufficient_samples')
        T = solve([s for s in samples if s['split'] == 'train'])
        result.update(complete=True, T_hand_camera=T.tolist())
        (directory/'grid/extrinsics.yaml').write_text(yaml.safe_dump({
            'parent_frame': config['frames']['hand'], 'child_frame': config['frames']['camera'],
            'convention': 'T_hand_camera maps camera points into hand',
            'method': METHOD, 'T_hand_camera': T.tolist()}, sort_keys=False))
    except (ValueError, cv2.error) as error:
        result['error'] = str(error)
    write_json(directory/'calibration.json', result)
    return result['complete']


def evaluate(directory):
    directory = Path(directory)
    result = json.loads((directory/'calibration.json').read_text())
    if not result['complete']:
        raise ValueError('AprilGrid calibration is incomplete')
    samples = json.loads((directory/'grid/samples.json').read_text())
    truth = {s['sample_id']: s for s in json.loads((directory/'grid/truth.json').read_text())}
    T = np.asarray(result['T_hand_camera'])
    rows = []
    for sample in samples:
        if sample['split'] != 'test':
            continue
        actual = truth[sample['sample_id']]
        ext = errors(T, np.asarray(actual['T_hand_camera']))
        predicted = np.asarray(sample['T_base_hand']) @ T @ np.asarray(sample['T_camera_target'])
        loc = errors(predicted, np.asarray(actual['T_base_board']))
        rows.append({'sample_id': sample['sample_id'], 'reprojection_px': sample['reprojection_px'],
                     **{'extrinsic_'+k: v for k, v in ext.items()},
                     **{'target_'+k: v for k, v in loc.items()}})
    result['test'] = {k: stats([row[k] for row in rows]) for k in rows[0] if k != 'sample_id'}
    with (directory/'evaluation_samples.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    write_json(directory/'evaluation.json', result)
    report(directory)


def report(directory):
    directory = Path(directory)
    path = directory/'evaluation.json'
    result = json.loads((path if path.exists() else directory/'calibration.json').read_text())
    trials = json.loads((directory/'grasps.json').read_text()) if (directory/'grasps.json').exists() else []
    expected = len(yaml.safe_load((directory/'config.yaml').read_text())['grasp']['trials'])
    good = sum(t['outcome'] == 'success' for t in trials)
    full = result['complete'] and len(trials) == expected
    lines = ['# AprilGrid 手眼标定与深度抓取', '',
             '固定组合：4×4 AprilGrid + OpenCV Daniilidis + Isaac SingleViewDepthCameraSensor（50 mm 基线）。',
             '抓取使用配准后的带噪声深度；理想深度与仿真位姿仅用于评分。结果不代表真实 D435 精度。', '',
             f"标定采样：{result['counts']}；标定完成：{result['complete']}。", '',
             '|指标|RMSE|P95|最大值|', '|---|---|---|---|']
    metrics = dict(result.get('test', {}))
    metrics.update({'TCP_'+k: stats([t['tcp_error'][k] for t in trials if 'tcp_error' in t])
                    for k in ('translation_mm', 'rotation_deg')})
    for k, v in metrics.items():
        if v['count']:
            lines.append(f"|{k}|{v['rmse']:.4f}|{v['p95']:.4f}|{v['maximum']:.4f}|")
    lines += ['', f'物理抓取成功：**{good}/{len(trials)}**（计划 {expected} 次）。',
              f'完整物理实验完成：{full}', '',
              '验收：抬升指令 100 mm，物体实际抬升至少 80 mm，保持 2 秒，TCP 相对滑移不超过 10 mm。', '',
              '失败原因：' + json.dumps({k: sum(t['outcome'] == k for t in trials)
                  for k in ('perception_failure', 'planning_failure', 'not_grasped', 'dropped')}, ensure_ascii=False)]
    for k in ('valid_fraction', 'depth_bias_mm', 'depth_rmse_mm', 'plane_rmse_mm'):
        lines += ['', k + '：`' + json.dumps(stats([t['depth'][k] for t in trials if k in t.get('depth', {})])) + '`']
    if 'error' in result:
        lines += ['', '标定失败：' + result['error']]
    metadata = directory/'metadata.json'
    if metadata.exists() and json.loads(metadata.read_text()).get('calibration_depth_registration', '').startswith('legacy'):
        lines += ['', '历史标定帧附带未配准深度，仅归档，未参与标定或抓取；抓取帧使用已配准的传感器深度。']
    with (directory/'grasps.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['trial', 'outcome', 'reason', 'tcp_translation_mm', 'tcp_rotation_deg'])
        writer.writeheader()
        for t in trials:
            writer.writerow({'trial': t['trial'], 'outcome': t['outcome'], 'reason': t.get('reason', ''),
                             **{'tcp_'+k: t.get('tcp_error', {}).get(k, '') for k in ('translation_mm', 'rotation_deg')}})
    scores = directory/'evaluation_samples.csv'
    if scores.exists():
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        with scores.open() as f:
            rows = list(csv.DictReader(f))
        fig, axes = plt.subplots(1, 2, figsize=(9, 3))
        axes[0].plot([float(r['target_translation_mm']) for r in rows], 'o-')
        axes[0].set_title('Held-out target position')
        axes[1].plot([t['tcp_error']['translation_mm'] for t in trials if 'tcp_error' in t], 'o-')
        axes[1].set_title('Pre-close TCP position')
        for ax in axes:
            ax.set_xlabel('Trial'); ax.set_ylabel('Error (mm)')
        fig.tight_layout(); fig.savefig(directory/'errors.png'); plt.close(fig)
        lines += ['', '![精度](errors.png)']
    lines += ['', '原始数据与统计：grid/、grasp_frames/、evaluation.json、evaluation_samples.csv、grasps.json、grasps.csv。']
    (directory/'report.md').write_text('\n'.join(lines))
