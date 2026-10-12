"""OpenAI-compatible model settings, independent of ROS for offline validation."""
from dataclasses import dataclass
import os


@dataclass(frozen=True)
class ModelConfig:
    backend: str
    base_url: str
    api_key: str
    agent_model: str
    vision_model: str
    max_tokens: int = 512

    @classmethod
    def from_env(cls, env=None):
        env = os.environ if env is None else env
        backend = env.get('LLM_BACKEND', 'cloud')
        if backend not in ('local', 'cloud'):
            raise ValueError('LLM_BACKEND must be local or cloud')
        local = backend == 'local'
        model = env.get('AGENT_MODEL') or ('embodied-qwen-local' if local else 'qwen3.6-flash')
        return cls(backend, env.get('AGENT_API_BASE') or (
            'http://127.0.0.1:8000/v1' if local else 'https://dashscope.aliyuncs.com/compatible-mode/v1'),
            (env.get('AGENT_API_KEY') or 'local') if local else (
                env.get('AGENT_API_KEY') or env.get('DASHSCOPE_API_KEY', '')),
            model, env.get('VL_MODEL') or model)

    def completion_options(self):
        if self.backend == 'local':
            return dict(max_tokens=self.max_tokens, temperature=0.2,
                        extra_body={'chat_template_kwargs': {'enable_thinking': False}})
        return {}

    def history(self, history):
        if self.backend == 'cloud':
            return history
        # Keep a bounded recent conversation with room for tools and image tokens.
        return [{**item, 'content': item.get('content', '')[-800:]}
                for item in history[-4:] if item.get('role') in ('user', 'assistant')]


def create_client(config):
    from openai import OpenAI, DefaultHttpxClient
    options = {}
    if config.backend == 'local':
        # Never send loopback inference through a desktop HTTP/SOCKS proxy.
        options['http_client'] = DefaultHttpxClient(trust_env=False)
    return OpenAI(api_key=config.api_key, base_url=config.base_url,
                  timeout=120.0, max_retries=0, **options)
