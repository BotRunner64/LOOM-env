# LOOM-env

基于 Isaac Lab 的机器人操作仿真与数据框架，面向 context-conditioned VLA 的轨迹生成、上下文构造和对照评测。

本体范围：双臂，每臂 6 或 7 自由度，夹爪单独描述。技术路线为 Isaac Lab → Isaac Sim → PhysX；专家通过有物理反馈的状态机调用 cuRobo。

任务、场景、部署分别配置，采集、重放和运动预览共用真实资产定义。首批场景使用 ManiSkill 木桌、RoboDojo 积木和收纳篮；提供抓起物体、放入容器两个任务。资产来源、固定版本、碰撞表示和功能标注集中维护，下载与转换产物放在 `.cache/assets/` 或共享目录。任务不依赖具体对象名称，环境不选择任务或专家，统一 Runner 负责执行与记录。

双 Panda 接入接触测量与任务专家。Piper、X5、UR5＋WSG、xArm6＋Robotiq、OpenArm、YAM 保留双臂运动、FK 和夹爪诊断接口，尚未接入抓取专家。当前物理验收范围见[实现说明](docs/implementation.md)。

- [当前实现与运行方式](docs/implementation.md)：资产准备、采集、重放和验证结果。
- [场景资产与扩展边界](docs/scene-assets.md)：来源筛选、质量检查、功能标注和扩展步骤。
- [本体支持与资产准备](docs/embodiments.md)：七类本体的来源、接口与验证范围。
- [架构设计](docs/architecture.md)：配置组合、Runner、Episode 协议与 Context 实验规划。
- [环境安装](docs/environment.md)：固定依赖与 GPU／仿真环境。

依赖统一在 `pyproject.toml` 声明：

```bash
conda create -n loom-env python=3.12 'pip>=25.1'
conda activate loom-env
pip install -e . --group dev
```

基础配置与数据测试不需要启动仿真，也不需要下载场景资产：

```bash
python scripts/inspect_data.py config configs/collection/pick_place.yaml
python scripts/inspect_data.py config configs/collection/lift.yaml
python -m pytest -q
ruff check .
```

运行仿真需完成[完整环境安装](docs/environment.md#完整仿真)和[资产准备](docs/implementation.md#资产准备)。
