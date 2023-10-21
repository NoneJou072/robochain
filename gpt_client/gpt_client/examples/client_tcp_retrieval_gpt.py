import argparse
import os
import json
import logging
import socket

from langchain.chains import RetrievalQA
from langchain.chat_models import ChatOpenAI

from gpt_client.gpt_client.prompts.prompt_template import QA_TEMPLATE_BAICHUAN
import gpt_client.gpt_client.commons.embedding_utils as eu
from gpt_client.gpt_client.commons.utils import colors, streaming_print_banner

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
    os.environ["OPENAI_API_BASE"] = keys["OPENAI_API_BASE"]
    os.environ["PINECONE_API_KEY"] = keys["PINECONE_API_KEY"]


class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, verbose=False) -> None:
        set_global_configs()

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


def main(args=None):
    IS_DUBUG = True
    gpt = GPTAssistant(
        verbose=True,
    )
    if not IS_DUBUG:
        HOST = 'localhost'
        PORT = 5001
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        print("Connected to server.")

    while True:
        question = input(colors.YELLOW + "User💬> " + colors.ENDC)
        if question == "!quit" or question == "!exit":
            break
        if question == "!clear":
            os.system("clear")
            continue

        result = gpt.ask(question)  # Ask a question
        print(colors.GREEN + "Assistant🤖> " + colors.ENDC + f"{result}")
        if not IS_DUBUG:
            s.sendall(question.encode())


if __name__ == '__main__':
    main()
