import argparse
import os
import json
import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT

from langchain.chains import ConversationChain
from langchain.chat_models import ChatOpenAI
from langchain.memory import ConversationBufferMemory
from gpt_client.prompt_template import MEMORY_CONVERSATION_TEMPLATE, MEMORY_CONVERSATION_TEMPLATE_2


class colors:
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"


class ChatConfig:
    def __init__(self) -> None:
        dir_path = os.path.dirname(__file__)
        parser = argparse.ArgumentParser()
        parser.add_argument("--openai_key", type=str, default=os.path.join(dir_path, "config.json"))
        self._args = parser.parse_args()

    @property
    def args(self):
        return self._args


class GPTAssistant:
    """
        Load ChatGPT config and your custom pre-prompts.
    """
    def __init__(self) -> None:
        print("Loading configs...")
        cfg_args = ChatConfig().args
        with open(cfg_args.openai_key, "r") as f:
            config = json.load(f)
        os.environ["OPENAI_API_KEY"] = config["OPENAI_API_KEY"]
        os.environ["SERPAPI_API_KEY"] = '6d2f006798271a8662686caf77bbc47d94994fd97579b439fe2f4ead2f54f6e2'
        print(f"Done.")

        print("Initialize langchain...")
        # 初始化memory
        history = ConversationBufferMemory()
        # 预制一个实例
        history.chat_memory.add_user_message("open the gripper!")
        history.chat_memory.add_ai_message(
            """
```python
pri.gripper_ctrl('open')
```

This code uses the `gripper_ctrl('open')` function to open the gripper to a target angle from the 
current angle. """
        )

        # 初始化LLM
        llm = ChatOpenAI(temperature=0.1, max_tokens=2048)

        # 初始化问答链
        self.conversation = ConversationChain(
            llm=llm,
            verbose=True,
            prompt=MEMORY_CONVERSATION_TEMPLATE_2,
            memory=history
        )

        print(f"Done.")
        os.system("clear")
        print(
            colors.BLUE + "Welcome to the SimBot! I am ready to help you with your questions and commands." + colors.ENDC)


class GPTClient(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("%s already." % node_name)
        self.gpt_client = self.create_client(GPT, "gpt_service")
        while not self.gpt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for the server to go online...')

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
        node.send_msg(result)

    node.destroy_node()
    rclpy.shutdown()
