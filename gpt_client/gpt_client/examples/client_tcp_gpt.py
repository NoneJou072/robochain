import os
import sys
import logging
import socket

from langchain.globals import set_debug
from langchain_openai import ChatOpenAI
from langchain_core.runnables import RunnableParallel, RunnablePassthrough
from langchain_core.output_parsers import StrOutputParser

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from gpt_client.gpt_client.prompts.prompt_template import *
import gpt_client.gpt_client.commons.embedding_utils as eu
from gpt_client.gpt_client.commons.utils import *

logging.basicConfig(level=logging.INFO)
IS_DUBUG = True


class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, IS_DUBUG=False) -> None:
        set_debug(IS_DUBUG)
        set_global_configs()

        logging.info("Initialize LLM...")
        model = ChatOpenAI(
            model="gpt-3.5-turbo",
            temperature=0.1,
            max_tokens=2048,
        )
        prompt = ZERO_FUNC_PROMPT
        logging.info(f"Done.")

        logging.info("Initialize tools...")
        # embedding_model = eu.init_embedding_model()
        # vector_store = eu.init_vector_store(embedding_model)
        # retriever = vector_store.as_retriever(search_kwargs={'k': 3})

        # setup_and_retrieval = RunnableParallel(
        #     {"context": retriever, "question": RunnablePassthrough()}
        # )
        output_parser = StrOutputParser()
        logging.info(f"Done.")

        logging.info("Initialize chain...")
        # self.chain = setup_and_retrieval | prompt | model | output_parser
        self.chain = prompt | model | output_parser

        logging.info(f"Done.")

        os.system("clear")
        streaming_print_banner()

    def ask(self, question):
        return self.chain.invoke({"input": question})


def main(args=None):
    gpt = GPTAssistant(
        IS_DUBUG=IS_DUBUG,
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
            s.sendall(result.encode())


if __name__ == '__main__':
    main()
