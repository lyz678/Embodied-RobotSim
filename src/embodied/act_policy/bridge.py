"""Agent-side process bridge. No torch/ROS sensor imports and no teacher fallback."""
import json
import os
import signal
import subprocess
import time
from pathlib import Path

HERE=Path(__file__).resolve().parent


def run_act_pick(node,class_id,progress_cb=None):
    cfg=json.loads((HERE/'config.json').read_text())
    model=os.environ.get('EMBODIED_ACT_MODEL','rgb')
    checkpoint=os.environ.get('EMBODIED_ACT_CHECKPOINT') or cfg['checkpoints'][model]
    output=HERE.parents[2]/'log/embodied_act'/f'pick_{time.time_ns()}'
    output.mkdir(parents=True)
    if not node.base_stopped():return False,'ACT未启动：底座未停稳'
    node.pub_vel.publish(__import__('geometry_msgs.msg',fromlist=['Twist']).Twist())
    script=HERE/'deploy.py'
    command=[cfg['ros_python'],str(script),'--class-id',str(class_id),'--output',str(output),'--model',model,'--checkpoint',checkpoint]
    message=f'ACT {model}抓取，类别={class_id}，权重={checkpoint}'
    node.get_logger().info(message)
    if progress_cb:progress_cb(message)
    env=dict(os.environ,PYTHONPATH=cfg['act_root']+os.pathsep+os.environ.get('PYTHONPATH',''),PYTHONUNBUFFERED='1')
    process=None
    try:
        with (output/'deployment.log').open('w') as log:
            process=subprocess.Popen(command,cwd=cfg['act_root'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            node._act_process=process
            process.wait(timeout=cfg['wall_timeout'])
        result=json.loads((output/'result.json').read_text())
        node.get_logger().info(f'ACT result {output}: {result}')
        if result['success']:
            return True,f'ACT完成闭爪、夹持回Home及2秒关节/接触检查；物体身份与实际抬起需独立评分。记录：{output}'
        return False,f'ACT失败：{result["failure_reason"]}；记录：{output}'
    except BaseException as exc:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        if not isinstance(exc,Exception):raise
        return False,f'ACT部署异常：{exc}；记录：{output}'
    finally:
        node._act_process=None


def stop_act_pick(node):
    process=getattr(node,'_act_process',None)
    if process and process.poll() is None:
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
