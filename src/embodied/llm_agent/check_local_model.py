#!/usr/bin/env python3
"""Smoke-test text, native tool calls and vision without commanding a robot."""
import argparse
import asyncio
from types import SimpleNamespace
import base64
import json
from pathlib import Path
import time
from openai import OpenAI
from model_config import ModelConfig, create_client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, required=True, help='JPEG or PNG camera snapshot')
    parser.add_argument('--output', type=Path, default=Path('.cache/qwen/validation.json'))
    parser.add_argument('--agent', action='store_true', help='Also exercise the real Agent loop with a fake robot (requires sourced ROS)')
    args = parser.parse_args()
    cfg = ModelConfig.from_env()
    if cfg.backend != 'local':
        parser.error('Set LLM_BACKEND=local')
    client = create_client(cfg)
    report = {'model': cfg.agent_model, 'checks': {}}
    started = time.monotonic()
    try:
        response = client.chat.completions.create(model=cfg.agent_model,
            messages=[{'role':'user','content':'用一句中文介绍你能如何协助机器人完成任务。'}], **cfg.completion_options())
        report['text'] = response.choices[0].message.content
        report['checks']['text'] = bool(report['text'])
        tools = [{'type':'function','function':{'name':'navigate_to_location','description':'导航到指定房间',
            'parameters':{'type':'object','properties':{'location':{'type':'string','enum':['厨房','客厅']}},'required':['location']}}}]
        messages = [{'role':'system','content':'你是机器人助手。用户要求导航时必须调用工具。'},
                    {'role':'user','content':'请导航到厨房。'}]
        response = client.chat.completions.create(model=cfg.agent_model, messages=messages,
            tools=tools, tool_choice='auto', **cfg.completion_options())
        msg = response.choices[0].message
        calls = msg.tool_calls or []
        report['tool_calls'] = [call.model_dump() for call in calls]
        report['checks']['native_tool_calls'] = len(calls)==1 and calls[0].function.name=='navigate_to_location' and json.loads(calls[0].function.arguments).get('location')=='厨房'
        if calls:
            messages.append(msg.model_dump(exclude_none=True))
            messages.extend({'role':'tool','tool_call_id':call.id,'content':'测试返回：已到达厨房。未实际控制机器人。'} for call in calls)
            response = client.chat.completions.create(model=cfg.agent_model, messages=messages,
                tools=tools, tool_choice='auto', **cfg.completion_options())
            report['tool_result_reply'] = response.choices[0].message.content
            report['checks']['tool_result_roundtrip'] = bool(report['tool_result_reply']) and not response.choices[0].message.tool_calls
        image = base64.b64encode(args.image.read_bytes()).decode()
        mime = 'image/png' if args.image.suffix.lower()=='.png' else 'image/jpeg'
        response = client.chat.completions.create(model=cfg.vision_model,
            messages=[{'role':'user','content':[{'type':'image_url','image_url':{'url':f'data:{mime};base64,{image}'}},
                {'type':'text','text':'用一句中文描述图片中实际可见的场景和物体，不要猜测不可见内容。'}]}], **cfg.completion_options())
        report['vision'] = response.choices[0].message.content
        report['checks']['vision_response'] = bool(report['vision'])
        if args.agent:
            from agent_server import Qwen3Agent

            class FakeRobot:
                def __init__(self):
                    self._node = SimpleNamespace(current_scene='客厅', localization_ready=lambda: True)
                    self.actions = []

                async def recognize_scene(self, client):
                    return '[客厅] 测试场景，不读取或控制机器人'

                async def navigate(self, location):
                    self.actions.append(('navigate', location))
                    return True, '模拟导航成功，未发送真实目标'

                async def stop(self):
                    self.actions.append(('stop',))

            async def check_agent():
                robot = FakeRobot()
                agent = Qwen3Agent(robot)
                records = []
                for prompt, expected in [('请立即停止机器人。', ('stop',)), ('请去厨房。', ('navigate', '厨房'))]:
                    events = []
                    async def send(event):
                        events.append(event)
                    await agent.chat(prompt, [], send)
                    records.append(dict(prompt=prompt, events=events))
                    if expected not in robot.actions or any(event['type']=='error' for event in events) or not any(event['type']=='assistant' for event in events):
                        return False, records
                return True, records
            passed, records = asyncio.run(check_agent())
            report['agent'] = records
            report['checks']['agent_loop_fake_robot'] = passed
    except Exception as exc:
        report['error'] = str(exc)
    report['wall_seconds'] = time.monotonic()-started
    report['passed'] = len(report['checks'])==(5 if args.agent else 4) and all(report['checks'].values()) and 'error' not in report
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
