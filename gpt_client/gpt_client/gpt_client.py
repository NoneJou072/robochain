import argparse
import os
import json
import logging
import time

import rclpy
from rclpy.node import Node
from gpt_interface.srv import GPT

from langchain.chat_models import ChatOpenAI
from langchain.memory import ConversationBufferMemory

from langchain.embeddings.openai import OpenAIEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Pinecone
from langchain.document_loaders import TextLoader
import pinecone

from langchain.chains import ConversationChain
from langchain.chains import RetrievalQA

from gpt_client.prompt_template import QA_TEMPLATE, MEMORY_CONVERSATION_TEMPLATE_2

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
    os.environ["SERPAPI_API_KEY"] = keys["SERPAPI_API_KEY"]
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
        logging.info("Initialize LLM...")
        llm = ChatOpenAI(
            model="gpt-3.5-turbo",
            temperature=0.1,
            max_tokens=2048,
        )
        logging.info(f"Done.")

        # Initialize chat tools
        logging.info("Initialize tools...")
        if mode=='memory':
            # Initialize memory
            logging.info("Initialize memory...")
            memory = ConversationBufferMemory()

        elif mode=='retriever':
            # Initialize vector store
            logging.info("Initialize vector store...")
            loader = TextLoader(os.path.join(os.path.dirname(__file__), "prompts/task_settings.txt"))
            documents = loader.load()
            text_splitter = CharacterTextSplitter(separator="---", chunk_size=1000, chunk_overlap=0)
            docs = text_splitter.split_documents(documents)

            embeddings = OpenAIEmbeddings()

            pinecone.init(api_key=os.getenv("PINECONE_API_KEY"), environment="us-west4-gcp-free")
            index_name = "langchain-demo"
            # Check if our index already exists. If it doesn't, we create it
            if index_name not in pinecone.list_indexes():
                # we create a new index
                pinecone.create_index(
                    name=index_name,
                    metric='cosine',
                    dimension=1536  
                )
            # The OpenAI embedding model `text-embedding-ada-002 uses 1536 dimensions`
            # vector_store = Pinecone.from_documents(docs, embeddings, index_name=index_name)

            # if you already have an index, you can load it like this
            vector_store = Pinecone.from_existing_index(index_name, embeddings)
        else:
            raise ValueError('Invalid mode.')
        logging.info(f"Done.")

        # Initialize QA-chain
        logging.info("Initialize chain...")
        if mode=='memory':
            self.conversation = ConversationChain(
                llm=llm,
                verbose=verbose,
                prompt=MEMORY_CONVERSATION_TEMPLATE_2,
                memory=memory
            )
        elif mode=='retriever':
            chain_type_kwargs = {"prompt": QA_TEMPLATE, "verbose":verbose}
            self.conversation = RetrievalQA.from_chain_type(
                llm=llm,
                chain_type='stuff',
                retriever=vector_store.as_retriever(search_kwargs={'k': 1}),
                chain_type_kwargs=chain_type_kwargs,
                return_source_documents=verbose
            )
        else:
            raise ValueError('Invalid mode.')
        logging.info(f"Done.")

        os.system("clear")
        with open(os.path.join(os.path.dirname(__file__), 'banner.txt'), 'r') as f:
            banner = f.read()
            streaming_print(banner, color=colors.YELLOW)
        streaming_print("I am ready to help you with your questions and commands.\n", color=colors.BLUE)
    
    def ask(self, question):
        if self.mode=='memory':
            result = self.conversation.predict(input=question)
        elif self.mode=='retriever':
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
        is_debug=False,
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
