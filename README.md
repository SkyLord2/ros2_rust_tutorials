# ROS 2 教程

这是一个基于 ROS 2 和 Rust/C++/Python 的教程项目，帮助你快速了解如何在 ROS 2 环境中开发节点、创建消息与服务，并完成常见的 ROS 2 集成任务。

## 项目简介

本仓库包含一系列示例，覆盖以下内容：

- ROS 2 节点开发
- 发布与订阅话题
- 服务与客户端调用
- 参数管理
- 依赖与构建配置
- 常见开发流程与实践

## 环境要求

- ROS 2（推荐 Humble 或 Jazzy）
- Rust 工具链（推荐稳定版）
- colcon 构建工具
- 适当的 ROS 2 工作空间

## 快速开始

1. 创建或进入 ROS 2 工作空间
2. 将该仓库克隆到 `src` 目录下
3. 安装依赖
4. 构建项目：

```bash
colcon build --packages-select ros2_rust_tutorials
```

5. 运行示例节点：

```bash
source install/setup.bash
```

## 目录结构

```text
ros2_rust_tutorials/
├── README.md
├── src/
│   └── ...
├── msg/
│   └── ...
├── srv/
│   └── ...
├── CMakeLists.txt
├── package.xml
└── Cargo.toml
```

## 常用命令

### 构建

```bash
colcon build
```

### 运行节点

```bash
ros2 run <package_name> <node_name>
```

### 查看话题

```bash
ros2 topic list
```

### 查看服务

```bash
ros2 service list
```

## 备注

该 README 适合用于项目入门和基础学习，后续可以根据具体示例内容进一步补充详细使用说明。

## 许可证

请查看项目中的许可证文件，或在需要时自行补充。
