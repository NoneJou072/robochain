import os
import sys
import logging
import socket

import torch

from langchain.llms.huggingface_pipeline import HuggingFacePipeline
from langchain.chains import RetrievalQA

from transformers import AutoModelForCausalLM, AutoTokenizer, GenerationConfig
from transformers import pipeline

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from gpt_client.gpt_client.prompts.prompt_template import QA_TEMPLATE_BAICHUAN, QA_TEMPLATE_ZEPHYR
import gpt_client.gpt_client.commons.embedding_utils as eu
from gpt_client.gpt_client.commons.utils import *

logging.basicConfig(level=logging.INFO)


class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, model_id, verbose=False) -> None:
        logging.info("Loading keys...")
        cfg_file = os.path.join(os.path.dirname(__file__), '../commons/config.json')
        set_global_configs(cfg_file)
        logging.info(f"Done.")

        # Initialize chat model
        logging.info("Loading Model...")
        model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            torch_dtype=torch.bfloat16,
            device_map='auto',
            cache_dir='../cache/',
        )
        logging.info(f"Done.")

        logging.info("Initialize LLM...")
        try:
            generation_config = GenerationConfig.from_pretrained(model_id)
        except OSError:
            logging.warning(OSError)
            generation_config = None
        
        tokenizer = AutoTokenizer.from_pretrained(model_id, trust_remote_code=True, cache_dir='../cache/')
        # response, history = model.chat(tokenizer, "你好", history=[])
        # print(response)
        pipe = pipeline(
            "text-generation",
            model=model,
            # torch_dtype=torch.bfloat16,
            # device_map='auto',
            max_length=2048,
            tokenizer=tokenizer,
            generation_config=generation_config,
            model_kwargs={'temperature': 0.0}
        )
        llm = HuggingFacePipeline(pipeline=pipe)
        print(llm('hello'))
        # logging.info(f"Done.")

        # logging.info("Initialize tools...")
        # embedding_model = eu.init_embedding_model()
        # vector_store = eu.init_vector_store(embedding_model)
        # logging.info(f"Done.")

        # logging.info("Initialize chain...")
        # chain_type_kwargs = {"prompt": QA_TEMPLATE_ZEPHYR, "verbose": verbose}
        # self.conversation = RetrievalQA.from_chain_type(
        #     llm=llm,
        #     chain_type='stuff',
        #     retriever=vector_store.as_retriever(search_kwargs={'k': 3}),
        #     chain_type_kwargs=chain_type_kwargs,
        #     return_source_documents=True
        # )
        # logging.info(f"Done.")

        # os.system("clear")
        # streaming_print_banner()

    # def ask(self, question):
    #     result_dict = self.conversation(question)
    #     result = result_dict['result']
    #     return result


def main(args=None):
    IS_DUBUG = True
    gpt = GPTAssistant(
        model_id='THUDM/chatglm3-6b',
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
    