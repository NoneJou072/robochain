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
将大模型（OpenAI-GPT3.5） 与 ROS2（Foxy） 通讯框架结合，方便机器人开发人员快速使用大模型进行开发。

---

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
4. 编译
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
```bash
# Terminal 1
ros2 run gpt_server gpt_server
```
```bash
# Terminal 2
ros2 run gpt_client gpt_client
```

## License
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
