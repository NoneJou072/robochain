<h1 align="center">欢迎使用 ROS2-GPT-Interface 👋</h1>  

--- 

[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Foxy-brightgreen)](http://docs.ros.org/en/foxy/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-20.04-yellow)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/License-MIT-informational)](https://nonejou072.github.io/)
&nbsp;

> 基于 ROS2 与 ChatGPT 的仿真框架，用于实现大模型时代下的机器人交互任务  
> [English](README.md) | 中文文档

## 简介

---

将大模型（OpenAI-GPT3.5） 与 ROS2（Foxy） 通讯框架结合，加入提示词（prompts）与任务原语
（primitives）的使用，方便机器人开发人员快速使用大模型进行开发。


## 快速部署

--- 

1. 新建 ros 工作空间并进入空间中
   ```commandline
    mkdir gpt_ws && cd gpt_ws
    ```
2. 克隆本仓库到工作空间
    ```
    git clone https://github.com/NoneJou072/ROS2-GPT-Interface.git
   ```
3. 将仓库文件夹名称修改为 src, 然后安装相关依赖
    ```
    pip install -r src/requirements.txt
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. 编译，检查报错
    ```
   colcon build --symlink-install
   ```
   
## 使用

---
### 1. 修改配置
a. 修改`gpt_client/gpt_client/config.json`文件，替换为你的 openai-key
   ```
   "OPENAI_API_KEY": "<Your openai-key>"
   ```

### 2. 运行
分别在两个终端中启动服务端和客户端，等待客户端初始化完成后，
我们可以在终端内输入请求或问题，等待服务端执行或回应。
```bash
# Terminal 1
ros2 run gpt_server gpt_server
```
```bash
# Terminal 2
ros2 run gpt_client gpt_client
```

<div style="display: flex;">
  <div style="flex: 50%;">
    <img src="docs/assets/client_test.png" alt="Image 1">
  </div>
  <div style="flex: 54%;">
    <img src="docs/assets/server_test.png" alt="Image 2">
  </div>
</div>

### 3. 额外命令

| 命令     | 描述   |
|--------|------|
| !exit  | 退出进程 |
| !quit  | 退出进程 |
| !clear | 清屏   |

### 4. 提示（Prompt）
参考 Microsoft 的 [PromptCraft-Robotics](https://github.com/microsoft/PromptCraft-Robotics)，
我们内置了少量基本的机器人提示词，这些提示词存放在 `gpt_client/prompts` 中。根据这些提示词，
GPT 能识别我们的指令，并在服务端中转换成可以被执行的 Python 代码进行执行。开发者们可以添加自己
的提示词，让机器人能够根据笼统的描述执行相应的任务。 每个任务提示词包含的任务原语实例化存放在 
`gpt_server/primitives.py` 中。  

**关于 Prompts 的初始化：** 通常，为了制作符合我们需求的 GPT ，并能够顺利生成可以被提取的 Python 代码，
我们需要在初始化时通过 `system` 的角色类型进行格式化，然后再交替输入 `user` 和 `assistant` 角色的信息。具体地，可以查看
 `gpt_client/system_prompts/system.txt`。

## License

---

```
Copyright (c) 2023 Round Dolphiiin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```
