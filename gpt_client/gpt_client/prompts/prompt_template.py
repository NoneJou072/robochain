import os

try:
    from langchain_core.prompts.chat import PromptTemplate
except:
    from langchain_core.prompts import PromptTemplate
from langchain_core.prompts.chat import (
    ChatPromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate
)

PROMPTS_DIR = os.path.dirname(__file__)


class PromptLoader:
    @property
    def pri_prompt(self) -> str:
        with open(os.path.join(PROMPTS_DIR, "primitives.txt")) as f:
            prompt = f.read()
        return prompt

    @property
    def sce_prompt(self) -> str:
        with open(os.path.join(PROMPTS_DIR, "scene.txt")) as f:
            prompt = f.read()
        return prompt

    @property
    def sys_prompt(self) -> str:
        with open(os.path.join(PROMPTS_DIR, "system.txt")) as f:
            prompt = f.read()
        return prompt
    
    @property
    def settings_prompt(self) -> str:
        with open(os.path.join(PROMPTS_DIR, "task_settings.txt")) as f:
            prompt = f.read()
        return prompt
    

B_INST, E_INST = "[INST]", "[/INST]"
B_SYS, E_SYS = "<<SYS>>\n", "\n<</SYS>>\n\n"

_ROBOT_PROMPT_TEMPLATE = f"""
{PromptLoader().sys_prompt}

{PromptLoader().sce_prompt}

{PromptLoader().pri_prompt}

"""

_TASK_SETTINGS_PROMPT_TEMPLATE = f"""
Here are some rules you need to note:

{PromptLoader().settings_prompt}
----------
"""

# 第一种 memory prompt template 构建方法
_DEFAULT_MEMORY_CONVERSATION_TEMPLATE = B_INST + B_SYS + _ROBOT_PROMPT_TEMPLATE + _TASK_SETTINGS_PROMPT_TEMPLATE + E_SYS + """
Use the above context to answer the user's question and perform the user's command.
-----------
Current conversation:
{history}
Last line:
Human: {input}
You:""" + E_INST

MEMORY_CONVERSATION_TEMPLATE = PromptTemplate(
    input_variables=["history", "input"],
    template=_DEFAULT_MEMORY_CONVERSATION_TEMPLATE,
)

# 第二种 memory prompt template 构建方法
system_template = B_SYS + _ROBOT_PROMPT_TEMPLATE + _TASK_SETTINGS_PROMPT_TEMPLATE + E_SYS + """
Use the above context to answer the user's question and perform the user's command.
-----------
Current conversation:
{history}
"""
# 构建初始 messages 列表，这里可以理解为是 openai 传入的 messages 参数
messages = [
    SystemMessagePromptTemplate.from_template(system_template),
    HumanMessagePromptTemplate.from_template(B_INST + '{input}' + E_INST)
]
MEMORY_CONVERSATION_TEMPLATE_2 = ChatPromptTemplate.from_messages(messages)

# 构建用于 RQA 的 prompt template
_DEFAULT_QA_TEMPLATE = B_INST + B_SYS + _ROBOT_PROMPT_TEMPLATE + "{context}" + E_SYS + """
Use the above context to answer the user's question and perform the user's command.
-----------
Human: {question}
You:""" + E_INST

QA_TEMPLATE = PromptTemplate(
    input_variables=["context", "question"],
    template=_DEFAULT_QA_TEMPLATE,
)

# 构建用于 RQA 的 prompt template
_DEFAULT_QA_TEMPLATE = B_INST + B_SYS + _ROBOT_PROMPT_TEMPLATE + "{context}" + E_SYS + """
Use the above context to answer the user's question and perform the user's command.
-----------
{chat_history}
Human: {question}
You:""" + E_INST

QA_MEM_TEMPLATE = PromptTemplate(
    input_variables=["context", "chat_history", "question"],
    template=_DEFAULT_QA_TEMPLATE,
)

# 构建 Pure (用于 BAICHUAN/GPT) 的 RQA prompt template
_DEFAULT_QA_TEMPLATE_BAICHUAN = _ROBOT_PROMPT_TEMPLATE + "{context}" + """
Use the above context to answer the user's question and perform the user's command.
-----------
Human: {question}
You:"""

QA_TEMPLATE_BAICHUAN = PromptTemplate(
    input_variables=["context", "question"],
    template=_DEFAULT_QA_TEMPLATE_BAICHUAN,
)

# 构建用于 MISTRAL 的 QA prompt template
_DEFAULT_QA_TEMPLATE_MISTRAL = B_INST + _ROBOT_PROMPT_TEMPLATE + "{context}" + """
Use the above context to answer the user's question and perform the user's command.
-----------
Human: {question}
You:""" + E_INST

QA_TEMPLATE_MISTRAL = PromptTemplate(
    input_variables=["context", "question"],
    template=_DEFAULT_QA_TEMPLATE_MISTRAL,
)

# 构建用于 Zephyr 的 QA prompt template
_DEFAULT_QA_TEMPLATE_ZEPHYR = '<|system|>\n' + "{context}" + """
Use the above context to answer the user's question and perform the user's command.
""" + "\n<|user|>\n{question}""" + """\n<|assistant|>"""

QA_TEMPLATE_ZEPHYR = PromptTemplate(
    input_variables=["context", "question"],
    template=_DEFAULT_QA_TEMPLATE_ZEPHYR,
)

# 构建 ZERO_FUNC 的 prompt
with open(os.path.join(PROMPTS_DIR, "zero-func.txt")) as f:
    system_prompt = f.read()

ZERO_FUNC_PROMPT = ChatPromptTemplate.from_messages([
    SystemMessagePromptTemplate.from_template(system_prompt),
    ("user", "{input}")
])
