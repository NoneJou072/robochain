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

from gpt_client.gpt_client.prompts.prompt_template import QA_TEMPLATE
import gpt_client.gpt_client.commons.embedding_utils as eu
from gpt_client.gpt_client.commons.utils import colors, streaming_print_banner, set_global_configs

logging.basicConfig(level=logging.INFO)


class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, model_id: str = None, verbose=False) -> None:

        logging.info("Loading keys...")
        cfg_file = os.path.join(os.path.dirname(__file__), '../commons/config.json')
        set_global_configs(cfg_file)
        logging.info(f"Done.")

        logging.info("Initialize langchain...")
        
        # Initialize chat model
        model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            load_in_8bit=True,
            torch_dtype=torch.float16,
            device_map='auto',
            cache_dir='../cache/',
        )
        tokenizer = AutoTokenizer.from_pretrained(model_id, use_fast=False, trust_remote_code=True)

        logging.info("Initialize pipeline...")
        generation_config = GenerationConfig.from_pretrained(model_id)
        pipe = pipeline(
            "text-generation",
            model=model,
            torch_dtype=torch.bfloat16,
            device_map='auto',
            max_length=2048,
            repetition_penalty=1.15,
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
        chain_type_kwargs = {"prompt": QA_TEMPLATE, "verbose":verbose}
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
    IS_DUBUG = False
    if not IS_DUBUG:
        HOST = 'localhost'
        PORT = 2542
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        print("Connected to server.")

    gpt = GPTAssistant(
        model_id='codellama/CodeLlama-13b-Instruct-hf',
        verbose=True,
    )

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
    