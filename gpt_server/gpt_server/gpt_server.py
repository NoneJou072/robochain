import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpt_interface.srv import GPT
import re


class GPTServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.add_ints_server_ = self.create_service(GPT, "gpt_service", self.gpt_callback) 
        self.gpt_message = None
        self.code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
            
    def gpt_callback(self,request, response):
        self.gpt_message = request.data
        self.get_logger().info(self.gpt_message)
        return response
    
    def extract_python_code(self, content):
        code_blocks = self.code_block_regex.findall(content)
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
    node.get_logger().info("gpt_server.")
    print(f"Initializing Mujoco...")
    # aw = AirSimWrapper()
    print(f"Done.")
    while rclpy.ok():
        if node.gpt_message is not None:
            code = node.extract_python_code(node.gpt_message)
            if code is not None:
                print("Please wait while I run the code in AirSim...")
                exec(node.extract_python_code(node.gpt_message))
                print("Done!\n")
        rclpy.spin_once(node)
    rclpy.shutdown() 

