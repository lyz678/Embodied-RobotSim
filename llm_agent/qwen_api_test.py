import os
import base64
from openai import OpenAI

def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

client = OpenAI(
    # 若没有配置环境变量，请用阿里云百炼API Key将下行替换为：api_key="sk-xxx",
    # 各地域的API Key不同。获取API Key：https://help.aliyun.com/zh/model-studio/get-api-key
    api_key=os.getenv("DASHSCOPE_API_KEY"),
    # 各地域配置不同，请根据实际地域修改
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)

image_path = "/home/lyz/robotSim/assets/Pick&Place.gif"
base64_image = encode_image(image_path)

completion = client.chat.completions.create(
    model="qwen3.6-flash", # 此处以qwen3.6-flash为例，可按需更换模型名称。模型列表：https://help.aliyun.com/zh/model-studio/models 
    messages=[
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/gif;base64,{base64_image}"
                    },
                },
                {"type": "text", "text": "图中描绘的是什么景象?"},
            ],
        },
    ],
)
print(completion.choices[0].message.content)