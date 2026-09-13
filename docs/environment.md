# 开发与仿真环境

项目要求 **Isaac Sim >=6**，不限制小版本；Isaac Lab 按上游兼容关系选择。目标运行平台是 Linux x86_64、NVIDIA RTX GPU，物理后端为 PhysX，图像渲染使用 Isaac RTX。

项目直接依赖只在 `pyproject.toml` 中声明。普通库、PyTorch、Warp 不重复固定精确版本，由 Sim / Lab 的依赖关系决定。当前使用 Python 3.12 创建仿真环境；项目自身只要求 Python >=3.12，Sim 的 wheel 仍有自己的 Python ABI 限制。

## 创建环境

从仓库根目录执行。Conda 只提供 Python，依赖使用 pip 安装：

```bash
conda create -n loom-env python=3.12 'pip>=25.1'
conda activate loom-env
```

已有同名环境时先检查其安装来源。旧环境曾使用 `.deps/IsaacLab` 的五个 editable 子包和手动补丁；验证新安装方案应创建独立环境，避免这些旧包掩盖缺失依赖或覆盖官方 wheel 的模块。

## 安装依赖

### 基础开发

数据处理、配置和非仿真模块开发只需：

```bash
pip install -e . --group dev
```

`-e .` 使源码修改直接生效；`--group dev` 安装 pytest 和 Ruff，不需要开发工具时可省略。此命令不安装仿真依赖。

### 完整仿真

```bash
pip install -e '.[sim]' --group dev \
    --index-url https://pypi.org/simple \
    --extra-index-url https://pypi.nvidia.com \
    --extra-index-url https://download.pytorch.org/whl/cu128
pip check
```

[NVIDIA 源](https://pypi.nvidia.com/isaaclab/)提供官方 Isaac Lab 统一 wheel，已包含资产、PhysX、Omniverse 和可视化模块，无需克隆 Lab、安装五个源码子包或修改第三方源码。`isaaclab[isaacsim]>=3.0.0b2` 允许所需 API 系列的 Beta wheel，并由其 `isaacsim` extra 选择配套 Sim；项目额外表达的 Sim 要求只有 `>=6`。这不保证任意 Lab 与任意 Sim 都能混用。

PyTorch 下载源选择当前 Linux x86_64 仿真组合使用的 CUDA 12.8 构建；Torch 的具体版本由上游决定。若以后切换上游组合，应按其平台要求调整下载源，而不是在项目中再维护一套版本表。

cuRobo 从 `pyproject.toml` 指定的上游提交安装。保留这一源码提交是因为已发布的 v0.8.0 缺少小网格碰撞查询修复；该提交包含上游修复，不再应用本地补丁。后续有包含修复的正式发行版时可切换到发行包。

## 运行与验证

宿主机需要可供容器访问的 NVIDIA RTX GPU、图形驱动和 Vulkan ICD，以及 GLIBC >=2.35、Vulkan/OpenGL/EGL 运行库。Git 用于获取 cuRobo；资产获取和源码扩展可能还需要 git-lfs、C/C++ 工具链及匹配的 CUDA Toolkit。视频导出使用 `imageio-ffmpeg`。

从仓库根目录、激活环境后执行：

```bash
python scripts/check_env.py --curobo
```

检查项目依赖范围、`pip check`、数据读写、CUDA 矩阵运算、cuRobo 正向运动学及梯度，以及小网格的远处无碰撞和近处碰撞。结果写入 `.cache/checks/report.json`，子进程日志在同目录。版本检查通过不能替代仿真和实际任务验收。

使用 Isaac Sim 前需接受 [NVIDIA Omniverse EULA](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html)。同意后：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
# 仅以 root 运行的容器需要。
export OMNI_KIT_ALLOW_ROOT=1
python scripts/check_env.py --sim
```

`--sim` 创建本地几何体，执行 GPU PhysX 步进，检查方块落地高度及 64×64 RGB 相机输出；无需机器人资产。成功时保存 `.cache/checks/simulation-rgb.png`。headless 渲染仍依赖 GPU 图形驱动。实际任务的资产准备、采集和重放见[运行指南](implementation.md)。

## 当前节点与排查

当前节点使用 RTX 4090。若 Vulkan 未选择 NVIDIA ICD，在运行仿真前设置本节点路径：

```bash
export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json
```

该路径属于宿主机配置，不是所有机器的通用要求。`nvidia-smi` 顶部的 CUDA 版本是驱动支持上限；实际 PyTorch CUDA 构建可用 `python -c 'import torch; print(torch.version.cuda)'` 查看。

- 安装找不到 Lab Beta wheel：检查 NVIDIA 下载源是否可访问；仅使用 PyPI 可能得到旧版 Lab。
- 依赖冲突：先运行 `pip check`，检查是否混入旧 editable 子包；不要用 `--no-deps` 或修改第三方源码绕过正式安装的依赖解析。
- cuRobo 小网格检查失败：核对是否安装了项目声明的上游提交，不能仅看 `0.8.0` 版本前缀。
- 仿真启动失败：查看 `simulation.log`，检查 GPU、Vulkan ICD 和 EULA 设置；首次 RTX 着色器编译可能较慢。

## 验证记录

2026-09-13：移除项目对普通依赖的精确版本锁定，并用 NVIDIA 官方 Lab wheel 和包含碰撞修复的上游 cuRobo 验证无补丁安装。验证产物位于 `.cache/setup/no-patches/`，不提交 Git。

验证使用 `.cache/setup/no-patches/wheel-venv`，复用原环境的 Sim / GPU 运行库，独立安装官方 Lab wheel 和原版 cuRobo；已核对所有 Lab 子模块均来自 wheel，未加载旧 editable 源码。原 Conda 环境未迁移，此次也不等同于从空机器完成全部下载的验证。

实际组合为 Sim 6.0.1.0、Lab 3.0.0b2.post1、PyTorch 2.11.0+cu128、cuRobo 0.8.0.post1.dev43。它们是本次验证记录，不是项目新增的精确版本约束。

- 标准完整 pip 安装成功，报告：`install-plan.json`，日志：`full-install.log`。
- 依赖范围、`pip check`、数据读写、CUDA、cuRobo 正向运动学和梯度全部通过，报告：`final-checks/report.json`。
- 小网格远处碰撞代价为 0，近处为约 0.0075，梯度有限。
- GPU PhysX 方块落地中心高度为 0.0500 m，RTX 输出有效的 64×64 RGB；图像：`wheel-sim/simulation-rgb.png`，日志：`wheel-sim.log`。
- 148 项项目测试通过，Ruff 和文档命令语法检查通过。

默认 Panda 抓放（`configs/collection/pick_place.yaml`、seed 0）在原环境和无补丁环境中均执行 500 步后超时，物体落在篮子外；无补丁轨迹的数据结构检查通过。该案例未通过任务验收，不能把环境检查通过解释为抓放成功。两次物体位置轨迹最大分量差约 0.053 m，单次对照也不能证明新旧环境物理效果等价。对照报告为 `task-comparison.json`，运行日志为 `baseline.log` 和 `collection.log`，最终图像为 `panda-final.png`。

复现本次无补丁抓放检查（仓库根目录，先设置上文 EULA / Vulkan 变量）：

```bash
.cache/setup/no-patches/wheel-venv/bin/python scripts/collect.py \
    --collection configs/collection/pick_place.yaml \
    --output-dir outputs/environment-validation --episode-id panda-no-patches --seed 0
```

该命令使用本机保留的验证环境；新机器完成上文正式安装后使用 `python`。重复运行时更换 episode ID 或输出目录。成功与否以 `RESULT` 的 `outcome` 和实际轨迹为准。

历史上 2026-09-08 的 Sim 6.0.1 / Lab 源码标签组合通过了 GPU PhysX 和 RTX 相机检查，但带有本地补丁；这份历史结果不替代当前无补丁组合的验证。
