import os
import sys
import logging

import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT

from langchain.chains import RetrievalQA
from langchain_openai import ChatOpenAI

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from gpt_client.prompts.prompt_template import QA_TEMPLATE_BAICHUAN
import gpt_client.commons.embedding_utils as eu
from gpt_client.commons.utils import *

logging.basicConfig(level=logging.INFO)


class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, verbose=False) -> None:

        logging.info("Loading keys...")
        cfg_file = os.path.join(os.path.dirname(__file__), '../commons/config.json')
        set_global_configs(cfg_file)
        logging.info(f"Done.")

        logging.info("Initialize LLM...")
        llm = ChatOpenAI(
            model="gpt-3.5-turbo",
            temperature=0.1,
            max_tokens=2048,
        )
        logging.info(f"Done.")

        logging.info("Initialize tools...")
        embedding_model = eu.init_embedding_model()
        vector_store = eu.init_vector_store(embedding_model)
        logging.info(f"Done.")

        logging.info("Initialize chain...")
        chain_type_kwargs = {"prompt": QA_TEMPLATE_BAICHUAN, "verbose": verbose}
        self.conversation = RetrievalQA.from_chain_type(
            llm=llm,
            chain_type='stuff',
            retriever=vector_store.as_retriever(search_kwargs={'k': 3}),
            chain_type_kwargs=chain_type_kwargs,
            return_source_documents=True
        )
        logging.info(f"Done.")

        os.system("clear")
        streaming_print_banner()
    
    def ask(self, question):
        result_dict = self.conversation(question)
        result = result_dict['result']
        return result


class GPTClient(Node):
    """ ROS2 client node.
        
        node_name(str): client node's name
        is_debug(bool): if True, there will be no interaction with the server.
        mode(str): using 'retriever'/'memory' to build the chain.                                                                                                                                     
        verbose:(bool): whether to print the whole context on the terminal.
    """
    def __init__(self, node_name: str, is_debug: bool=False, verbose=False) -> None:
        super().__init__(node_name)
        self.get_logger().info("%s already." % node_name)
        self.gpt_client = self.create_client(GPT, "gpt_service")

        self.is_debug = is_debug
        if not self.is_debug:
            while not self.gpt_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Waiting for the server to go online...')

        self.gpt = GPTAssistant(
            verbose=verbose,
        )

    def result_callback(self, result):
        response = result.result()

    def send_msg(self, msg):
        """ Send messages to server. """
        
        request = GPT.Request()
        request.data = msg
        self.gpt_client.call_async(request).add_done_callback(self.result_callback)

    def ask(self, question):
        return self.gpt.ask(question)


def main(args=None):
    rclpy.init(args=args)
    os.environ["TOKENIZERS_PARALLELISM"] = "false"
    
    gpt_node = GPTClient(
        "gpt_client",
        is_debug=False,
        verbose=True
    )
    
    while rclpy.ok():
        question = input(colors.YELLOW + "User💬> " + colors.ENDC)
        if question == "!quit" or question == "!exit":
            break
        if question == "!clear":
            os.system("clear")
            continue

        result = gpt_node.ask(question)  # Ask a question
        print(colors.GREEN + "Assistant🤖> " + colors.ENDC + f"{result}")
        if not gpt_node.is_debug:
            gpt_node.send_msg(result)

    gpt_node.destroy_node()
    rclpy.shutdown()
