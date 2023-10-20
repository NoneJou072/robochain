import argparse
import os
import json
import logging
import socket

import torch

from langchain import HuggingFacePipeline
from langchain.chains import RetrievalQA

from transformers import AutoModelForCausalLM, AutoTokenizer, GenerationConfig, AutoConfig
from transformers import pipeline

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
    os.environ["PINECONE_API_KEY"] = keys["PINECONE_API_KEY"]


class GPTAssistant:
    """ Load ChatGPT config and your custom pre-prompts. """

    def __init__(self, verbose=False) -> None:
        logging.info("Loading keys...")
        set_global_configs()
        logging.info(f"Done.")

        logging.info("Initialize langchain...")

        # Initialize chat model
        model_id = 'Open-Orca/oo-phi-1_5'
        model = AutoModelForCausalLM.from_pretrained(
            model_id,
            trust_remote_code=True,
            torch_dtype=torch.bfloat16,
            device_map='auto',
            # load_in_8bit=True,
            cache_dir='../cache/',
            offload_folder="offload_folder"
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
    # main()
    from ctransformers import AutoModelForCausalLM
    from langchain.llms import CTransformers

    # Set gpu_layers to the number of layers to offload to GPU. Set to 0 if no GPU acceleration is available on your system.
    llm = AutoModelForCausalLM.from_pretrained("../cache/models--Mistral-7B-openorca/mistral-7b-instruct-v0.1.Q4_K_M.gguf",
                                               model_file="../cache/models--Mistral-7B-openorca/mistral-7b-instruct-v0.1.Q4_K_M.gguf",
                                               model_type="mistral", gpu_layers=50, context_length=2048)
    # config = {'max_new_tokens': 256, 'repetition_penalty': 1.1}
    # llm = CTransformers(
    #     model="../cache/models--Mistral-7B-openorca/mistral-7b-instruct-v0.1.Q4_K_M.gguf",
    #     model_file="../cache/models--Mistral-7B-openorca/mistral-7b-instruct-v0.1.Q4_K_M.gguf",
    #     model_type="mistral", gpu_layers=50, config=config, context_length=2048)
    prompt = """
### Instruction:
You are an assistant helping me control the robots.

When I ask you to do something, you are supposed to give me Python code that can achieve the task by using prepared functions.

You are only allowed to use the functions I have defined for you, don't declare or define any functions such as `def main()` by yourself.

You are not to use any other hypothetical functions or libraries that you think might exist.

You can use simple Python functions from libraries such as math and numpy.

If you don't know the answer or can't understand the commands, say you don't, don't try to make it up.

You dont't need to generate code explanations.

You are working in a room, where exists a robotic arm with a two-finger gripper on its end, and a workbench with 
three different colored blocks randomly placed on it, respectively called 'red_block', 'white_block' and 'green_block'.
Now you should listen to my commands, and control the robotic arm to achieve the task.

Here are some functions you can use to command the robot:

pri.gripper_ctrl(command) - Ctrl the gripper, param 'command' is a string type, consist of 'open' and 'close'.

pri.move(position, quaternion) - Move the end-effector/gripper to the target position and quaternion. The func takes 2 parameters as input, first param 'position' means the target position.
    second param 'quaternion' means the target quaternion. Both of them are type ndarray. 

pri.get_current_pose() -> position, quaternion - Get current pose of the gripper, This func will return 2 parameters, the position and quaternion of the end gripper.

pri.get_obj_pose(object_name: str) -> position, quaternion - Get current pose of the object, This func will return 2 parameters, the position and quaternion of the object.
example: `object_pos, object_quat = pri.get_obj_pose("object_name")`

pri.reset_robot() - Reset the robot to initial pose.

pri.grab(object_name: str) - Grab specified object according to the input name.

---

Here is an example need followed of the answering:

Human: open the gripper!
You: 
```python
pri.gripper_ctrl('open')
```

If you want to let the robot grab or pick up an object, You need to follow the following process: 
1. grab the specified object.
2. called `reset_robot()` func to reset to initial pose.

---

After grabed something, if you want to let the robot put the object a specified pose, You need to follow the following process: 
1. Get and calculate the target pose where the object is to be placed.
2. Move the gripper to the target pose.
3. Open the gripper.
Here is an example for you, if you want to put the object down 10cm to the front of the green block, you need first get the pose of green block, and 
then calculate the target pose such as `target_pos = green_pos + np.array([.1, .0, .0])`, it means the position 10cm in front of the block. Finally
move to this target pose and open the gripper.
---

The units are calculated in meters. If centimeters are used, they need to be converted to meters. 
Almostly, move above... or move up... means you need let the robotic gripper move positive direction along the z-axis.
pose means the position and quaternion, the quaternion expressed in 'w,x,y,z'.

Use the above context to answer the user's question and perform the user's command.
-----------
Human: Pick up the red block, then put it down 10cm up to the blue block.
You:
### Response:
    """
    print(llm(prompt))

    print('done')
