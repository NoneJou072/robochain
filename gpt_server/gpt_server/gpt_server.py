import os
import sys
import re
import time
import logging
import numpy as np

logging.basicConfig(level=logging.INFO)


COMMUNICATION_MODE = "ROS2"  # "ROS2" or "TCP"
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
import demo_env


class GPTServer(Node):
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

    def extract_python_code(self, content):
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

    def execute_python_code(self, pri, code):
        """ Execute python code with the input content.

        :param pri(Class): class name in prompts.
        :param code(str): python method to call.
        """
        print("\033[32m" + "Please wait while I run the code in Sim..." + "\033[m")
        print("\033[32m" + "code:" + "\033[m" + code)
        try:
            exec(code)
        except Exception as e:
            self.get_logger().warn("Found error while running the code: {}".format(e))
        print("\033[32m" + "Done!\n" + "\033[m")


def main(args=None):
    logging.info(f"Initializing ROS...")
    rclpy.init(args=args)
    node = GPTServer("gpt_server")
    node.get_logger().info("Gpt server has init.")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    logging.info(f"Done.")
    
    logging.info(f"Initializing Simulator...")
    env = demo_env.make_env()
    env.reset()
    logging.info(f"Done.")

    while rclpy.ok():
        if isinstance(node.code, str):
            node.execute_python_code(env, node.code)
            node.code = None

        env.step(env.action)

        if env.render_mode=='human':
            env.render()

        if not executor.spin_once(timeout_sec=0.001):
            time.sleep(0.001)  # Sleep for a short duration before checking again

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
