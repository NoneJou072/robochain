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
from langchain.chains import ConversationChain

from transformers import AutoModelForCausalLM, AutoTokenizer, GenerationConfig, AutoConfig
from transformers import pipeline

from gpt_client.gpt_client.prompts.prompt_template import MEMORY_CONVERSATION_TEMPLATE_2, MEMORY_CONVERSATION_TEMPLATE
from gpt_client.commons.utils import *

logging.basicConfig(level=logging.INFO)


class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, verbose=False) -> None:
        logging.info("Loading keys...")
        cfg_file = os.path.join(os.path.dirname(__file__), '../commons/config.json')
        set_global_configs(cfg_file)
        logging.info(f"Done.")
        
        # Initialize chat model
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
            repetition_penalty=1.15,
            pad_token_id=2,
            tokenizer=tokenizer,
            generation_config=generation_config,
        )

        logging.info("Initialize LLM...")
        llm = HuggingFacePipeline(
            pipeline=pipe,
        )
        logging.info(f"Done.")

        logging.info("Initialize memory...")
        memory = ConversationBufferMemory()
        logging.info(f"Done.")

        logging.info("Initialize chain...")
        self.conversation = ConversationChain(
            llm=llm,
            verbose=verbose,
            prompt=MEMORY_CONVERSATION_TEMPLATE,
            memory=memory
        )
        logging.info(f"Done.")

        os.system("clear")
        streaming_print_banner()

    
    def ask(self, question):
        result = self.conversation.predict(input=question)
        return result
