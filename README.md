<h1 align="center">Welcome to ROS2-GPT-Interface 👋</h1>  

--- 

[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Foxy-brightgreen)](http://docs.ros.org/en/foxy/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-20.04-yellow)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/License-MIT-informational)](https://nonejou072.github.io/)
&nbsp;

> A simulation framework based on ROS2 and ChatGPT for robot interaction tasks in the era of large models
> English | [中文文档](README-CN.md)

## Introduction

---

Combining the large model (OpenAI-GPT3.5) with the ROS2 (Foxy) communication
framework makes it easy for robot developers to quickly use the large model
for development.

---

## Installation

--- 

1. Create a new ros workspace and enter the space
   ```commandline
    mkdir gpt_ws && cd gpt_ws
    ```
2. Clone the repo to the workspace
    ```
    git clone https://github.com/NoneJou072/ROS2-GPT-Interface.git
   ```
3. Change the name of the repo folder to src, and then install related dependencies
    ```
    pip install -r src/requirements.txt
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. build the workspace
    ```
   colcon build --symlink-install
   ```
   
## Usage

---
### 1. Change setting
a. rewrite `gpt_client/gpt_client/config.json` file，instead of your openai-key
   ```
   "OPENAI_API_KEY": "<Your openai-key>"
   ```

### 2. Run the node
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
