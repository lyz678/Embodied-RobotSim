#!/usr/bin/env python3
"""Wait for the configured OpenAI endpoint and verify the requested model exists."""
import json
import time
import urllib.request
from model_config import ModelConfig


def wait_ready(timeout=180):
    config = ModelConfig.from_env()
    deadline = time.monotonic()+timeout
    error = ''
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    while time.monotonic() < deadline:
        try:
            request = urllib.request.Request(config.base_url.rstrip('/')+'/models',
                                             headers={'Authorization': 'Bearer '+config.api_key})
            with opener.open(request, timeout=3) as response:
                models = json.load(response)['data']
            if config.agent_model not in [item['id'] for item in models]:
                raise ValueError(f'Model {config.agent_model} is not served by {config.base_url}')
            print(f'Local model ready: {config.agent_model}', flush=True)
            return
        except (OSError, ValueError, KeyError) as exc:
            error = str(exc)
            time.sleep(1)
    raise RuntimeError(f'Local model not ready after {timeout}s: {error}')


if __name__ == '__main__':
    wait_ready()
