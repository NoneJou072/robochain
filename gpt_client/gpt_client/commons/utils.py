import logging
import os
import time
import json


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

def streaming_print_banner():
    with open(os.path.join(os.path.dirname(__file__), 'banner.txt'), 'r') as f:
        banner = f.read()
        streaming_print(banner, color=colors.YELLOW)
    streaming_print("I am ready to help you with your questions and commands.\n", color=colors.BLUE)


class colors:
    RED = "\033[31m"
    ENDC = "\033[m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"


def set_global_configs(cfg_file: str = None):
    """ Load your app's keys and add to an enviroment variable. """
    os.environ["TOKENIZERS_PARALLELISM"] = "false"
    
    if cfg_file is None:
        cfg_file = os.path.join(os.path.dirname(__file__), 'config.json')
    logging.info("Loading keys...")
    with open(cfg_file, "r") as f:
        keys: dict = json.load(f)
        for key, value in keys.items():
            os.environ[key] = value
    logging.info(f"Done.")
