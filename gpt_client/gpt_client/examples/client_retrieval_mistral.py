import argparse
import os
import json
import logging

import torch

import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT

from langchain import HuggingFacePipeline
from langchain.chains import RetrievalQA

from transformers import AutoModelForCausalLM, AutoTokenizer, GenerationConfig, AutoConfig
from transformers import pipeline

from gpt_client.prompts.prompt_template import QA_TEMPLATE_BAICHUAN
import gpt_client.commons.embedding_utils as eu
from gpt_client.commons.utils import colors, streaming_print_banner

logging.basicConfig(level=logging.INFO)


def set_global_configs():
    """ Load your app's keys and add to an enviroment variable. """
    
    cfg_path = os.path.join(os.path.dirname(__file__), '../commons')
    parser = argparse.ArgumentParser()
    parser.add_argument("--keys_list", type=str, default=os.path.join(cfg_path, "config.json"))
    args = parser.parse_args()
    with open(args.keys_list, "r") as f:
        keys = json.load(f)
    os.environ["OPENAI_API_KEY"] = keys["OPENAI_API_KEY"]
    os.environ["PINECONE_API_KEY"] = keys["PINECONE_API_KEY"]

class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, verbose=False) -> None:

        logging.info("Loading keys...")
        set_global_configs()
        logging.info(f"Done.")

        logging.info("Initialize langchain...")
        
        # Initialize chat model
        model_id = 'mistralai/Mistral-7B-Instruct-v0.1'
        model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            torch_dtype=torch.bfloat16,
            device_map='auto',
            # load_in_8bit=True,
        )

        logging.info("Initialize pipeline...")
        generation_config = GenerationConfig.from_pretrained(model_id)
        tokenizer = AutoTokenizer.from_pretrained(model_id, use_fast=False, trust_remote_code=True)
        pipe = pipeline(
            "text-generation",
            model=model,
            # torch_dtype=torch.bfloat16,
            # device_map='auto',
            max_length=2048,
            # repetition_penalty=1.15,
            pad_token_id=2,
            tokenizer=tokenizer,
            generation_config=generation_config,
        )
        logging.info(f"Done.")

        logging.info("Initialize LLM...")
        llm = HuggingFacePipeline(pipeline=pipe)
        logging.info(f"Done.")

        logging.info("Initialize tools...")
        embedding_model = eu.init_embedding_model()
        vector_store = eu.init_vector_store(embedding_model)
        logging.info(f"Done.")

        logging.info("Initialize chain...")
        chain_type_kwargs = {"prompt": QA_TEMPLATE_BAICHUAN, "verbose":verbose}
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
    gpt_node = GPTClient(
        "gpt_client",
        is_debug=True,
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
