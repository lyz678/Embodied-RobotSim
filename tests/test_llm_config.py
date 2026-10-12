"""Check backend selection and local resource limits without ROS or a GPU."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/embodied/llm_agent'))
from model_config import ModelConfig


class ConfigTests(unittest.TestCase):
    def test_cloud_requires_real_key(self):
        self.assertEqual(ModelConfig.from_env({}).api_key, '')
        self.assertEqual(ModelConfig.from_env({'DASHSCOPE_API_KEY':'test'}).api_key, 'test')

    def test_local_does_not_use_cloud_secret(self):
        cfg = ModelConfig.from_env({'LLM_BACKEND':'local','DASHSCOPE_API_KEY':'cloud-secret'})
        self.assertEqual(cfg.api_key, 'local')
        self.assertEqual(cfg.base_url, 'http://127.0.0.1:8000/v1')
        self.assertEqual(cfg.agent_model, cfg.vision_model)
        self.assertFalse(cfg.completion_options()['extra_body']['chat_template_kwargs']['enable_thinking'])

    def test_endpoint_override(self):
        cfg = ModelConfig.from_env({'LLM_BACKEND':'local','AGENT_API_BASE':'http://localhost:9000/v1',
                                   'AGENT_MODEL':'custom','VL_MODEL':'vision','AGENT_API_KEY':'private'})
        self.assertEqual((cfg.agent_model,cfg.vision_model,cfg.api_key), ('custom','vision','private'))
        self.assertEqual(cfg.base_url, 'http://localhost:9000/v1')

    def test_local_history_is_bounded_and_original_unchanged(self):
        history = [{'role':'user','content':'a'*4000} for _ in range(40)]
        kept = ModelConfig.from_env({'LLM_BACKEND':'local'}).history(history)
        self.assertEqual(len(kept),4)
        self.assertEqual(len(kept[0]['content']),800)
        self.assertEqual(len(history[0]['content']),4000)

    def test_invalid_backend(self):
        with self.assertRaises(ValueError):
            ModelConfig.from_env({'LLM_BACKEND':'typo'})


if __name__ == '__main__':
    unittest.main()
