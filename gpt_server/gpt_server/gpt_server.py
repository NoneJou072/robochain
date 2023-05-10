import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT
import re
import os, sys
sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from primitives import PrimitiveSet
from rclpy.executors import SingleThreadedExecutor
import time


class GPTServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.add_ints_server_ = self.create_service(GPT, "gpt_service", self.gpt_callback) 
        self.code = None
            
    def gpt_callback(self,request, response):
        gpt_message = request.data
        self.get_logger().info(gpt_message)
        self.code = self.extract_python_code(gpt_message)
        response.res = True
        if self.code is None:
            response.res = False
        return response
    
    def extract_python_code(self, content):
        code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
        code_blocks = code_block_regex.findall(content)
        if code_blocks:
            full_code = "\n".join(code_blocks)

            if full_code.startswith("python"):
                full_code = full_code[7:]

            return full_code
        else:
            return None
    

def main(args=None):
    rclpy.init(args=args) 
    node = GPTServer("gpt_server") 
    node.get_logger().info("Gpt server has init.")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    print(f"Initializing Simulator...")
    imi = PrimitiveSet()
    print(f"Done.")

    while rclpy.ok():

        """
            <Write your main loop here.>
        """
        
        if node.code is not None:
            print("Please wait while I run the code in Sim...")
            print("code:", node.code)
            try:
                exec(node.code)
            except Exception as e:
                node.get_logger().warn("Found error while running the code: {}".format(e))
            print("Done!\n")
            node.code = None
            
        if not executor.spin_once(timeout_sec=0.001):
            time.sleep(0.001) # Sleep for a short duration before checking again
    rclpy.shutdown() 

