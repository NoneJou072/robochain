from typing import List
import openai
import re
import argparse
import os
import json
import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT

from langchain.document_loaders import UnstructuredFileLoader
from langchain.embeddings.openai import OpenAIEmbeddings
from langchain.vectorstores import Chroma
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.chains import ChatVectorDBChain, ConversationalRetrievalChain, ConversationChain
from langchain.chat_models import ChatOpenAI
from langchain.prompts.chat import (
  ChatPromptTemplate,
  SystemMessagePromptTemplate,
  HumanMessagePromptTemplate
)
from langchain.memory import ConversationBufferMemory
from langchain.schema import messages_from_dict, messages_to_dict


class colors: 
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"

class EnvConfig:
    def __init__(self) -> None:
        dir_path = os.path.dirname(__file__)
        parser = argparse.ArgumentParser()
        parser.add_argument("--openai_key", type=str, default=os.path.join(dir_path, "config.json"))
        parser.add_argument("--pri_prompt", type=str, default=os.path.join(dir_path, "prompts/primitives.txt"))
        parser.add_argument("--sce_prompt", type=str, default=os.path.join(dir_path, "prompts/scene.txt"))
        parser.add_argument("--sys_prompt", type=str, default=os.path.join(dir_path, "system_prompts/system.txt"))
        self._cfg = parser.parse_args()
    
    @property
    def cfg(self):
        return self._cfg


class GPTAssistant:
    """
        Load ChatGPT config and your custom pre-prompts.
    """
    def __init__(self) -> None:
        print("Loading configs...")
        cfg = EnvConfig().cfg

        with open(cfg.openai_key, "r") as f:
            config = json.load(f)
        openai.api_key = config["OPENAI_API_KEY"]
        os.environ["OPENAI_API_KEY"] = config["OPENAI_API_KEY"]
        os.environ["SERPAPI_API_KEY"] = '6d2f006798271a8662686caf77bbc47d94994fd97579b439fe2f4ead2f54f6e2'

        with open(cfg.sys_prompt, "r") as f:
            sys_prompt = f.read()
        with open(cfg.sce_prompt, "r") as f:
            sce_prompt = f.read()
        with open(cfg.pri_prompt, "r") as f:
            pri_prompt = f.read()

        system_template = cfg.sys_prompt + """
        Use the above context to answer the user's question.
        -----------
        {question}
        -----------
        {chat_history}
        """

        # 构建初始 messages 列表，这里可以理解为是 openai 传入的 messages 参数
        messages = [
            SystemMessagePromptTemplate.from_template(system_template),
            HumanMessagePromptTemplate.from_template('{question}')
        ]

        # 初始化历史消息
        history = ConversationBufferMemory()
        history.chat_memory.add_user_message("open robotic gripper!")
        history.chat_memory.add_ai_message(
        """```python
imi.openGripper(imi.target_gripper_angle)
```

This code uses the `openGripper(imi.target_gripper_angle)` function to open the gripper to a target angle from the current angle.""")
        # 初始化 prompt 对象
        # prompt = ChatPromptTemplate.from_messages(history.messages)

        # 初始化LLM
        llm = ChatOpenAI(temperature=0.1,max_tokens=2048)

        # 初始化问答链
        self.conversation = ConversationChain(
            llm=llm, 
            verbose=True,
            memory=history
        )

        print(f"Done.")
        os.system("clear")
        print(colors.BLUE + "Welcome to the SimBot! I am ready to help you with your questions and commands." + colors.ENDC)


class GPTClient(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("%s already." % node_name)
        self.gpt_client = self.create_client(GPT, "gpt_service")
        # while not self.gpt_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().warn('Waiting for the server to go online...')
    
    def result_callback(self, result):
        response = result.result()
    
    def send_msg(self, msg):
        """
            Send messages to server.
        """
        request = GPT.Request()
        request.data = msg
        self.gpt_client.call_async(request).add_done_callback(self.result_callback)


def main(args=None):
    rclpy.init(args=args)
    node = GPTClient("gpt_client")
    gpt = GPTAssistant()

    while rclpy.ok():
        question = input(colors.YELLOW + "User💬> " + colors.ENDC)
        if question == "!quit" or question == "!exit":
            break
        if question == "!clear":
            os.system("clear")
            continue
        
        # 发送问题
        result = gpt.conversation.predict(input=question)
        print(colors.GREEN + "Assistant🤖> " + colors.ENDC + f"{result}")
        # node.send_msg(response)
    
    node.destroy_node()
    rclpy.shutdown()
