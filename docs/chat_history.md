# 0. 关于 Chat History
在使用LLM（大语言模型）时，我们通常需要保存上下文的内容，例如
1. 在对话前加入 prompt，使 gpt 根据我们预先设置好的规则回答问题;
2. 在每次对话完成后，将用户的问题和 gpt 生成的回答加入聊天历史中，实现上下文内容保存。
参考链接：https://python.langchain.com/en/latest/modules/memory/getting_started.html?highlight=ChatMessageHistory#saving-message-history

在 LangeChian 中，有两种方式实现 chat-history，下面会进行介绍。

# 1. 使用列表保存历史消息
使用GPT3.5模型构建油管频道问答机器人
在 chatgpt api（也就是 GPT-3.5-Turbo）模型出来后，因钱少活好深受大家喜爱，所以 LangChain 也加入了专属的链和模型，我们来跟着这个例子看下如何使用他。
``` python
import os

from langchain.chains import ConversationalChain
from langchain.chat_models import ChatOpenAI
from langchain.prompts.chat import (
  ChatPromptTemplate,
  SystemMessagePromptTemplate,
  HumanMessagePromptTemplate
)

system_template = """
Use the following context to answer the user's question.
If you don't know the answer, say you don't, don't try to make it up. And answer in Chinese.
-----------
{context}
-----------
{chat_history}
"""

# 构建初始 messages 列表，这里可以理解为是 openai 传入的 messages 参数
messages = [
  SystemMessagePromptTemplate.from_template(system_template),
  HumanMessagePromptTemplate.from_template('{question}')
]

# 初始化 prompt 对象
prompt = ChatPromptTemplate.from_messages(messages)


# 初始化问答链
qa = ConversationalChain.from_llm(ChatOpenAI(temperature=0.1,max_tokens=2048),prompt=prompt)


chat_history = []
while True:
  question = input('问题：')
  # 开始发送问题 chat_history 为必须参数,用于存储对话历史
  result = qa({'question': question, 'chat_history': chat_history})
  chat_history.append((question, result['answer']))
  print(result['answer'])
```
# 2. 使用 memory 管理历史消息
## 2.1 在 chain 外使用 memory
使用Memory实现一个带记忆的对话机器人
上一个例子我们使用的是通过自定义一个列表来存储对话的方式来保存历史的。

当然，你也可以使用自带的 memory 对象来实现这一点。
```python
from langchain.memory import ChatMessageHistory
from langchain.chat_models import ChatOpenAI

chat = ChatOpenAI(temperature=0)

# 初始化 MessageHistory 对象
history = ChatMessageHistory()

# 给 MessageHistory 对象添加对话内容
history.add_ai_message("你好！")
history.add_user_message("中国的首都是哪里？")

# 执行对话
ai_response = chat(history.messages)
print(ai_response)
```
ChatMessageHistory 是这部分模块的基本接口，如果想在 chain 外记忆历史信息，
可以直接使用该类。否则应该使用其他类型的 memory 接口。

## 2.2 在 chain 中使用 memory
`ConversationBufferMemory`类是`ChatMessageHistory`的一个 wrapper，它
允许储存 messages。，并以变量的形式提取 messages。
使用 memory 后，我们无需手动将历史消息加入到下一次对话中，langchain 会为我们自动保存。

### 2.2.1 手动加入历史信息
``` python
from langchain.memory import ConversationBufferMemory
memory = ConversationBufferMemory()
```
方式一：
``` python
memory.chat_memory.add_user_message("hi!")
memory.chat_memory.add_ai_message("whats up?")
```
方式二：
``` python
memory.save_context({"input": "hi"}, {"output": "whats up"})
```

### 2.2.2 提取信息
以字符串的形式提取：
``` python
memory.load_memory_variables({})
```
```
{'history': 'Human: hi!\nAI: whats up?'}
```
以消息列表的方式提取：
``` python
memory = ConversationBufferMemory(return_messages=True)
memory.chat_memory.add_user_message("hi!")
memory.chat_memory.add_ai_message("whats up?")
memory.load_memory_variables({})
```
```
{'history': [HumanMessage(content='hi!', additional_kwargs={}),
  AIMessage(content='whats up?', additional_kwargs={})]}
```

## 2.3 保存历史消息到本地
langchain 为我们提供了保存和加载历史消息的方法，我们可以把消息类型转换成字典类型，或吧字典类型转换成消息类型。这样，我们就可以将字典类型的历史消息保存为 json 或其他格式，供下次加载使用。 
下面是两种类型的转换方法示例：

``` python
import json

from langchain.memory import ChatMessageHistory
from langchain.schema import messages_from_dict, messages_to_dict

history = ChatMessageHistory()

history.add_user_message("hi!")

history.add_ai_message("whats up?")
```

``` python
dicts = messages_to_dict(history.messages)
dicts
```

[{'type': 'human', 'data': {'content': 'hi!', 'additional_kwargs': {}}},
 {'type': 'ai', 'data': {'content': 'whats up?', 'additional_kwargs': {}}}]

``` python
new_messages = messages_from_dict(dicts)
new_messages
```

[HumanMessage(content='hi!', additional_kwargs={}),
 AIMessage(content='whats up?', additional_kwargs={})]

 # 3. 其他的 memory wrappers 介绍
 ## ConversationBufferWindowMemory
 实现滑动窗口的效果，即维护一个固定长度为 K 的交互列表，始终记忆最近的 K 个交互历史，这样缓存容器就不会过长。
 ``` python
 from langchain.memory import ConversationBufferWindowMemory

 memory = ConversationBufferWindowMemory( k=1)
 ```

 ## Entity Memory
 使用 memory 记忆有关指定的 entities（字符实体）的事情，即从字符实体内提取出关键信息，用于实时构建临时知识库。
 官方实例：https://python.langchain.com/en/latest/modules/memory/types/entity_summary_memory.html
 可以看见，根据我们聊天的内容，gpt 可以提取出关键主语，并为每个主语赋予其解释。

## 暂未更新