# LOOM-env

基于 Isaac Lab 的机器人操作仿真与数据框架，面向 context-conditioned VLA 的轨迹生成、上下文构造和对照评测。

本体范围：**双臂，每臂 6 或 7 自由度，夹爪单独描述**。

技术路线：**Isaac Lab → Isaac Sim → PhysX**。专家轨迹采用 **任务状态机 + cuRobo 运动规划 + 仿真执行验证**。

当前已完成架构设计和 Conda 环境搭建，Isaac Sim 6.0.1 的 GPU PhysX、RGB 渲染及 cuRobo 检查均已通过；任务框架尚未实现。

- [架构设计与实现计划](docs/architecture.md)：模块边界、核心数据协议、cuRobo 专家生成、状态恢复、Context 实验和分阶段验收。
- [环境安装与验证](docs/environment.md)：Conda 环境、pip 安装步骤和 GPU／仿真检查。

依赖统一在 [pyproject.toml](pyproject.toml) 声明：核心依赖、`sim` 可选依赖和 `dev` 开发依赖组。Conda 提供 Python 环境。

```bash
conda create -n loom-env python=3.12 'pip>=25.1'
conda activate loom-env
pip install -e . --group dev
```

以上安装基础开发依赖。完整仿真还需准备带补丁的 Isaac Lab 源码，再用 pip 安装 `sim` 可选依赖，见[完整仿真安装步骤](docs/environment.md#完整仿真)。
