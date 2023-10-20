import errno
import os
import sys
import re
import time
import logging
import numpy as np

logging.basicConfig(level=logging.INFO)

COMMUNICATION_MODE = "TCP"  # "ROS2" or "TCP"
if COMMUNICATION_MODE == "ROS2":
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from gpt_interface.srv import GPT
    except ImportError:
        logging.warning("ROS2 is not installed, please install ROS2 first.")
else:
    import socket

sys.path.append(os.path.dirname(__file__))
from demo_env import GraspingEnv


class GPTServer():
    """ GPT Server Node Class. """

    def __init__(self, name):
        super().__init__(name)
        self.add_ints_server_ = self.create_service(GPT, "gpt_service", self.gpt_callback)
        self.code = None

    def gpt_callback(self, request, response):
        gpt_message = request.data
        self.code = self.extract_python_code(gpt_message)
        response.res = True
        if self.code is None:
            response.res = False
        return response


def extract_python_code(content):
    """ Extract the python code from the input content.

    :param content: message contains the code from gpt's reply.
    :return(str): python code if the content is correct
    """
    code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
    code_blocks = code_block_regex.findall(content)
    if code_blocks:
        full_code = "\n".join(code_blocks)

        if full_code.startswith("python"):
            full_code = full_code[7:]

        return full_code
    else:
        return None


def execute_python_code(pri, code):
    """ Execute python code with the input content.

    :param pri(Class): class name in prompts.
    :param code(str): python method to call.
    """
    print("\033[32m" + "Please wait while I run the code in Sim..." + "\033[m")
    print("\033[32m" + "code:" + "\033[m" + code)
    try:
        exec(code)
    except Exception as e:
        logging.warning("Found error while running the code: {}".format(e))
    print("\033[32m" + "Done!\n" + "\033[m")


def main_tcp(args=None):
    logging.info(f"Initializing Simulator...")
    env = GraspingEnv()
    env.reset()
    logging.info(f"Done.")

    logging.info(f"Initializing TCP...")
    HOST = ''
    ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ss.bind((HOST, 5001))
    ss.listen(1)
    ss.setblocking(0)  # 设置为非阻塞
    logging.info(f"Done.")

    while True:
        try:
            logging.info(f"Waiting for connection...")
            conn, addr = ss.accept()
            conn.setblocking(0)  # 设置连接为非阻塞
            logging.info(f"Connected by {addr}")
            while True:
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    logging.info(f"Received: {data.decode()}")
                    code = extract_python_code(data.decode())
                    if code is not None:
                        execute_python_code(env, code)
                except socket.error as e:
                    if e.errno == errno.EWOULDBLOCK:
                        # 非阻塞套接字，没有数据可读时会引发 EWOULDBLOCK 错误
                        pass
                    else:
                        raise
                finally:
                    env.step(env.action)

        except socket.error as e:
            if e.errno == errno.EWOULDBLOCK:
                # 非阻塞套接字，没有连接时会引发 EWOULDBLOCK 错误
                pass
            else:
                raise
        # conn.close()


if __name__ == "__main__":
    main_tcp()
