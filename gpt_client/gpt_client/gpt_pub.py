#!~/anaconda3/bin/python
from typing import List
import openai
import re
import argparse
import numpy as np
import os
import json
import sys

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from functions import FuncIMI as imi
import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT


class GPTAssistant:
    def __init__(self) -> None:
        parser = argparse.ArgumentParser()
        parser.add_argument("--prompt", type=str, default=os.path.join(os.path.dirname(__file__), "prompts/basic.txt"))
        parser.add_argument("--sysprompt", type=str, default=os.path.join(os.path.dirname(__file__), "system_prompts/mj_basic.txt"))
        args = parser.parse_args()

        with open(os.path.join(os.path.dirname(__file__), "config.json"), "r") as f:
            config = json.load(f)

        print("Initializing ChatGPT...")
        openai.api_key = config["OPENAI_API_KEY"]

        with open(args.sysprompt, "r") as f:
            sysprompt = f.read()

        self.chat_history = [
            {
                "role": "system",
                "content": sysprompt
            },
            {
                "role": "user",
                "content": "open robotic gripper"
            },
            {
                "role": "assistant",
                "content": """```python
        imi.openGripper(imi.target_gripper_angle)
        ```

        This code uses the `openGripper(imi.target_gripper_angle)` function to open the gripper to a target angle from the current angle."""
            }
        ]
        print(f"Done.")
        with open(args.prompt, "r") as f:
            prompt = f.read()

        self.ask(prompt)
        print("Welcome to the Mujoco chatbot! I am ready to help you with your Mujoco questions and commands.")

    def ask(self, prompt):
        self.chat_history.append(
            {
                "role": "user",
                "content": prompt,
            }
        )
        completion = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.chat_history,
            temperature=0,
        )
        self.chat_history.append(
            {
                "role": "assistant",
                "content": completion.choices[0].message.content,
            }
        )
        return self.chat_history[-1]["content"]

    class colors:  # You may need to change color settings
        RED = "\033[31m"
        ENDC = "\033[m"
        GREEN = "\033[32m"
        YELLOW = "\033[33m"
        BLUE = "\033[34m"


class GPTClient(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("%s already." % node_name)
        self.gpt_client = self.create_client(GPT, "gpt_service")
    
    def result_callback(self, result):
        response = result.result()
    
    def send_msg(self, msg):
        request = GPT.Request()
        request.data = msg
        self.gpt_client.call_async(request).add_done_callback(self.result_callback)


def main(args=None):

    rclpy.init(args=args)
    node = GPTClient("node_gpt")
    gpt = GPTAssistant()
    while rclpy.ok():
        question = input(gpt.colors.YELLOW + "AirSim> " + gpt.colors.ENDC)

        if question == "!quit" or question == "!exit":
            break

        if question == "!clear":
            os.system("cls")
            continue

        response = gpt.ask(question)

        print(f"\n{response}\n")
        node.send_msg(response)
    
    rclpy.shutdown()
