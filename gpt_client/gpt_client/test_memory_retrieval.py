import argparse
import os
import json
import logging
import time

import torch

import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT

from langchain import HuggingFacePipeline
from langchain.memory import ConversationBufferMemory

from langchain.chains import ConversationChain, ConversationalRetrievalChain, LLMChain, StuffDocumentsChain
from langchain.chains import RetrievalQA

from transformers import AutoModelForCausalLM, AutoTokenizer, GenerationConfig, AutoConfig
from transformers import pipeline

from gpt_client.prompt_template import QA_TEMPLATE, MEMORY_CONVERSATION_TEMPLATE_2, MEMORY_CONVERSATION_TEMPLATE
import gpt_client.embedding_utils as eu

logging.basicConfig(level=logging.INFO)


def stream_string_generate(input_string):
    """ A python generator for stream printing. """
    for char in input_string:
        yield char
        time.sleep(0.005)  # Add a small delay for demonstration

def streaming_print(input, color: str):
    """ Print the input in a streaming manner. """
    char_generator = stream_string_generate(input)
    for char in char_generator:
        print(color + char + colors.ENDC, end='', flush=True)  # Use flush=True to force immediate printing


class colors:
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"


def set_global_configs():
    """ Load your app's keys and add to an enviroment variable. """
    
    dir_path = os.path.dirname(__file__)
    parser = argparse.ArgumentParser()
    parser.add_argument("--keys_list", type=str, default=os.path.join(dir_path, "config.json"))
    args = parser.parse_args()
    with open(args.keys_list, "r") as f:
        keys = json.load(f)
    os.environ["OPENAI_API_KEY"] = keys["OPENAI_API_KEY"]
    os.environ["PINECONE_API_KEY"] = keys["PINECONE_API_KEY"]

class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, verbose=False, mode='memory') -> None:
        
        self.mode = mode

        logging.info("Loading keys...")
        set_global_configs()
        logging.info(f"Done.")

        logging.info("Initialize langchain...")
        
        # Initialize chat model
        # model_id = 'meta-llama/Llama-2-7b-chat-hf'
        model_id = 'codellama/CodeLlama-13b-Instruct-hf'
        hf_auth = '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'  # Huggingface access token

        model_config = AutoConfig.from_pretrained(
            model_id,
            use_auth_token=hf_auth
        )
        model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            config=model_config,
            load_in_4bit=True,
            torch_dtype=torch.float32,
            device_map='auto',
            use_auth_token=hf_auth
        )
        tokenizer = AutoTokenizer.from_pretrained(model_id, use_auth_token=hf_auth)
        logging.info("Initialize generation config...")
        generation_config = GenerationConfig.from_pretrained(model_id, use_auth_token=hf_auth)
        logging.info("Initialize pipeline...")
        pipe = pipeline(
            "text-generation",
            model=model,
            torch_dtype=torch.bfloat16,
            device_map='auto',
            max_length=2048,
            # temperature=0.1,  # only used in sample-based generation modes.
            # top_p=0.95,
            repetition_penalty=1.15,
            pad_token_id=2,
            tokenizer=tokenizer,
            generation_config=generation_config,
        )

        logging.info("Initialize LLM...")
        llm = HuggingFacePipeline(
            pipeline=pipe,
            # model_kwargs={'temperature':0.1}
        )
        logging.info(f"Done.")

        # Initialize chat tools
        logging.info("Initialize tools...")
        if mode=='memory':
            # Initialize memory
            logging.info("Initialize memory...")
        memory = ConversationBufferMemory(memory_key='chat_history', return_messages=True, output_key='answer')

        if mode=='retriever':
            logging.info("Initialize vector store...")
            embedding_model = eu.init_embedding_model()
            vector_store = eu.init_vector_store(embedding_model)
        else:
            raise ValueError('Invalid mode.')
        logging.info(f"Done.")

        logging.info("Initialize chain...")
        if mode=='memory':
            self.conversation = ConversationChain(
                llm=llm,
                verbose=verbose,
                prompt=MEMORY_CONVERSATION_TEMPLATE,
                memory=memory,
            )
        elif mode=='retriever':
            chain_type_kwargs = {"prompt": QA_TEMPLATE, "verbose":verbose}
            # combine_docs_chain = StuffDocumentsChain(...)
            # question_generator_chain = LLMChain(llm=llm, prompt=QA_TEMPLATE)
            self.conversation = ConversationalRetrievalChain.from_llm(
                # combine_docs_chain=combine_docs_chain,
                llm=llm,
                # chain_type='stuff',
                retriever=vector_store.as_retriever(search_kwargs={'k': 3}),
                # chain_type_kwargs=chain_type_kwargs,
                # question_generator=question_generator_chain,
                return_source_documents=True,
                memory=memory,
                verbose=verbose,
                combine_docs_chain_kwargs={"prompt": QA_TEMPLATE},
            )
        logging.info(f"Done.")

        # os.system("clear")
        with open(os.path.join(os.path.dirname(__file__), 'banner.txt'), 'r') as f:
            banner = f.read()
            streaming_print(banner, color=colors.YELLOW)
        streaming_print("I am ready to help you with your questions and commands.\n", color=colors.BLUE)
    
    def ask(self, question):
        if self.mode=='memory':
            result = self.conversation.predict(input=question)
        elif self.mode=='retriever':
            result = self.conversation(question)
            # result = result_dict['result']
        return result


class GPTClient(Node):
    """ ROS2 client node.
        
        node_name(str): client node's name
        is_debug(bool): if True, there will be no interaction with the server.
        mode(str): using 'retriever'/'memory' to build the chain.                                                                                                                                     
        verbose:(bool): whether to print the whole context on the terminal.
    """
    def __init__(self, node_name: str, is_debug: bool=False, mode='memory', verbose=False) -> None:
        super().__init__(node_name)
        self.get_logger().info("%s already." % node_name)
        self.gpt_client = self.create_client(GPT, "gpt_service")

        self.is_debug = is_debug
        if not self.is_debug:
            while not self.gpt_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Waiting for the server to go online...')

        self.gpt = GPTAssistant(
            verbose=verbose,
            mode=mode
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
        mode='retriever',  # 'retriever' or 'memory'
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
