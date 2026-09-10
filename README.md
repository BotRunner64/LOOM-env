# LOOM-env

基于 Isaac Lab 的机器人操作仿真与数据框架，面向 context-conditioned VLA 的轨迹生成、上下文构造和对照评测。

本体范围：**双臂，每臂 6 或 7 自由度，夹爪单独描述**。

技术路线：**Isaac Lab → Isaac Sim → PhysX**。专家轨迹采用 **任务状态机 + cuRobo 运动规划 + 仿真执行验证**。

已实现双 Panda 的抓取放置闭环：`ManagerBasedEnv` 桌面场景、cuRobo 专家、真实接触反馈、统一 Runner、Episode 记录与物理动作重放。默认用一臂操作、另一臂保持，支持交换操作角色。部署统一配置前视、左腕、右腕三路 RGB，相机挂接与记录共用于采集和运动诊断。Piper、X5、UR5＋WSG、xArm6＋Robotiq、OpenArm 和 YAM 目前支持双臂运动、FK 与夹爪诊断；抓取专家尚未扩展到这些本体。

- [本体支持与资产准备](docs/embodiments.md)：七类本体的资产来源、准备命令、统一接口和验证范围。
- [当前实现与运行方式](docs/implementation.md)：采集与重放命令、数据格式和验证范围。
- [架构设计与实现计划](docs/architecture.md)：模块边界、核心数据协议、cuRobo 专家生成、状态恢复、Context 实验和分阶段验收。
- [环境安装与验证](docs/environment.md)：Conda 环境、pip 安装步骤和 GPU／仿真检查。

依赖统一在 [pyproject.toml](pyproject.toml) 声明：核心依赖、`sim` 可选依赖和 `dev` 开发依赖组。Conda 提供 Python 环境。

```bash
conda create -n loom-env python=3.12 'pip>=25.1'
conda activate loom-env
pip install -e . --group dev
```

以上安装基础开发依赖。完整仿真还需准备带补丁的 Isaac Lab 源码，再用 pip 安装 `sim` 可选依赖，见[完整仿真安装步骤](docs/environment.md#完整仿真)。

检查示例配置和基础链路（不启动仿真）：

```bash
python scripts/inspect_data.py config configs/collection/pick_place.yaml
python -m pytest -q
ruff check .
```

默认 collection 使用双 Panda，运动预览通过 `--deployment configs/deployments/dual_x5.yaml` 等部署文件切换本体。配置按任务、部署、场景分别维护，collection 用相对路径引用它们。大资产及转换产物放在已忽略的 `.cache/assets/` 或仓库外的共享目录。
