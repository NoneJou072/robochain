import os
import logging

import torch

from langchain import HuggingFacePipeline
from langchain.memory import ConversationBufferMemory

from langchain.chains import ConversationalRetrievalChain

from transformers import AutoModelForCausalLM, AutoTokenizer, GenerationConfig
from transformers import pipeline

from gpt_client.gpt_client.prompts.prompt_template import QA_TEMPLATE, MEMORY_CONVERSATION_TEMPLATE_2, MEMORY_CONVERSATION_TEMPLATE
import gpt_client.gpt_client.commons.embedding_utils as eu
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
        logging.info("Loading Model...")
        model_id = 'codellama/CodeLlama-13b-Instruct-hf'
        model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            load_in_4bit=True,
            torch_dtype=torch.float32,
            device_map='auto',
        )
        logging.info(f"Done.")
    
        logging.info("Initialize LLM...")
        generation_config = GenerationConfig.from_pretrained(model_id)
        tokenizer = AutoTokenizer.from_pretrained(model_id)
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
        llm = HuggingFacePipeline(pipeline=pipe)
        logging.info(f"Done.")

        logging.info("Initialize tools...")
        memory = ConversationBufferMemory(memory_key='chat_history', return_messages=True, output_key='answer')

        logging.info("Initialize vector store...")
        embedding_model = eu.init_embedding_model()
        vector_store = eu.init_vector_store(embedding_model)
        logging.info(f"Done.")

        logging.info("Initialize chain...")
        # question_generator_chain = LLMChain(llm=llm, prompt=QA_TEMPLATE)
        self.conversation = ConversationalRetrievalChain.from_llm(
            llm=llm,
            retriever=vector_store.as_retriever(search_kwargs={'k': 3}),
            # question_generator=question_generator_chain,
            return_source_documents=True,
            memory=memory,
            verbose=verbose,
            combine_docs_chain_kwargs={"prompt": QA_TEMPLATE},
        )
        logging.info(f"Done.")
    
    def ask(self, question):
        result = self.conversation(question)
        # result = result_dict['result']
        return result
