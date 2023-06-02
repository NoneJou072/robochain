import re
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from gpt_interface.srv import GPT
from roboimi.demos.demo_grasping import MotionPlanEnv
import numpy as np
import os
import sys
import roboimi.utils.KDL_utils.transform as T

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from primitives import PrimitiveSet


class GPTServer(Node):
    """
        GPT Server Node Class.
    """
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
    print(f"Initializing Simulator...")

    from roboimi.assets.robots.diana_med import DianaMed

    env = MotionPlanEnv(
        robot=DianaMed(),
        renderer="mujoco_viewer",
        is_render=True,
        control_freq=200,
        is_interpolate=True,
        is_pd=True
    )
    env.reset()
    print(f"Done.")

    print(f"Initializing ROS...")
    rclpy.init(args=args)
    node = GPTServer("gpt_server")
    node.get_logger().info("Gpt server has init.")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    print(f"Done.")

    Primitive = PrimitiveSet()
    init_pos, init_rot = env.kdl_solver.getEeCurrentPose(env.robot.single_arm.arm_qpos)
    print("Test primitive...")
    env.move(init_pos, env.can_quat['ceramic_can'])
    print("Done.")

    # import threading
    # cam_thread = threading.Thread(target=env.camera_viewer)
    # cam_thread.start()
    
    while rclpy.ok():
        if node.code is not None:
            node.execute_python_code(env, node.code)
            node.code = None

        """
            <Write your main loop here.>
        """

        env.step(env.action)
        if env.is_render:
            env.render()

        if not executor.spin_once(timeout_sec=0.001):
            time.sleep(0.001)  # Sleep for a short duration before checking again

    node.destroy_node()
    rclpy.shutdown()
