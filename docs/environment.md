# 开发与仿真环境

环境名：`loom-env`。目标平台：Linux x86_64、Python 3.12、NVIDIA RTX GPU。采用 **Isaac Sim 6.0.1 + Isaac Lab 3.0 Beta 2 Patch 1**，物理后端为 PhysX，图像渲染使用 Isaac RTX。

## 固定版本

| 组件 | 版本 | 说明 |
| --- | --- | --- |
| Python | 3.12 | Isaac Sim 6.0.1 的 Python ABI |
| Isaac Sim | 6.0.1.0，`all,extscache` | 包含运行扩展和扩展缓存 |
| Isaac Lab | `v3.0.0-beta2.patch1` | 提交 `ffff603eafc6b74264a5261cc0183d6a65390d78`，含一处下述兼容补丁 |
| PyTorch / torchvision / torchaudio | 2.11.0 / 0.26.0 / 2.11.0，均为 `+cu128` | 遵循发布的 `isaacsim-core` wheel 精确依赖 |
| cuRobo | v0.8.0 | 提交 `4ea77366ca48ee453e7df139e39fa6532af49f3b`，v2 API、`cu12` 依赖 |
| NumPy / SciPy | 2.3.1 / 1.17.0 | Isaac Sim 6.0.1 的精确依赖 |
| Warp | 1.13.0 | Isaac Lab 所选标签的精确依赖 |

按用户选择，优先使用 Isaac Sim 6.0.1。与其配套的 Lab 标签仍标注 Beta；截至 2026-09-08，Lab 2.3.2.post1 稳定包配套的是 Sim 5.1 / Python 3.11。本环境固定上述 Beta 发布标签，不跟随开发分支滚动更新。Lab 源码中各子包有独立版本号，`pip show isaaclab` 显示的核心包版本为 `6.1.14`，不等于仓库发布标签。

版本依据：[Sim 6.0.1 安装说明](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_python.html)、[Lab 配套发布说明](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-beta2.patch1)、[Sim 核心包依赖元数据](https://pypi.org/pypi/isaacsim-core/6.0.1.0/json)、[cuRobo v0.8.0](https://github.com/NVlabs/curobo/releases/tag/v0.8.0)。

## 创建环境

在项目根目录执行；已有环境时只需激活：

```bash
conda create -n loom-env python=3.12 'pip>=25.1'
conda activate loom-env
```

Conda 提供 Python，项目依赖统一由 `pyproject.toml` 声明，使用 pip 安装。pip 会自动准备隔离的构建环境。

## 安装依赖

### 基础开发

只做数据处理或基础模块开发时：

```bash
pip install -e . --group dev
```

`-e .` 安装当前项目，源码修改直接生效；`--group dev` 安装 pytest 和 Ruff，不需要开发工具时可省略。此命令不安装仿真依赖，`import loom_env` 不启动仿真。

### 完整仿真

先下载指定版本的 Isaac Lab 并应用兼容补丁。已有该版本源码且已应用补丁时，跳过这两条命令：

```bash
git clone --depth 1 --branch v3.0.0-beta2.patch1 https://github.com/isaac-sim/IsaacLab.git .deps/IsaacLab
git -C .deps/IsaacLab apply ../../patches/isaaclab-sim601-coverage.patch
```

补丁解决所选版本组合的依赖冲突：Lab 要求 `coverage==7.6.1`，Sim 要求 `coverage==7.4.4`。它将 Lab 的约束放宽为 `>=7.4.4,<8`，由 Sim 固定最终版本，不改仿真逻辑。升级 Lab 时需重新检查是否仍需要该补丁。

然后直接安装项目、仿真依赖和开发工具，无需先安装上节的基础依赖：

```bash
pip install -e '.[sim]' --group dev \
    -e .deps/IsaacLab/source/isaaclab \
    -e .deps/IsaacLab/source/isaaclab_assets \
    -e .deps/IsaacLab/source/isaaclab_physx \
    -e .deps/IsaacLab/source/isaaclab_ov \
    -e .deps/IsaacLab/source/isaaclab_visualizers \
    --index-url https://pypi.org/simple \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    --extra-index-url https://pypi.nvidia.com
pip check
```

`.[sim]` 读取 `pyproject.toml` 中的仿真依赖；五个本地路径提供 Lab 核心、资产、PhysX、Omniverse 和可视化子包；两个额外下载源提供 CUDA 版 PyTorch 和 NVIDIA 包。pip 在同一次解析中检查这些依赖的兼容性。Lab 以 editable 方式安装，请保留 `.deps/IsaacLab`，该目录不提交到仓库。

当前固定的 cuRobo 版本还需要 [小网格距离查询补丁](../patches/curobo-small-mesh-sdf.patch)。原实现把查询上限设为网格包围半径；对于小于机器人碰撞球的小物体，远处查询也可能误报碰撞。补丁保留配置的最小查询距离，并使用 Warp 的公开设备转换接口，不改资产几何或 PhysX 配置。安装后执行一次，重装 cuRobo 后重新应用：

```bash
patch -d "$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')" -p1 < patches/curobo-small-mesh-sdf.patch
python scripts/check_env.py --curobo
```

`--curobo` 包含小网格的远处无碰撞、近处碰撞及梯度检查。

## 宿主机依赖

运行仿真需要可供容器访问的 NVIDIA RTX GPU、图形驱动与 Vulkan ICD，以及 GLIBC >=2.35、Vulkan/OpenGL/EGL 运行库。源码获取需要 `git` 和 `git-lfs`；源码扩展编译需要 C/C++ 工具链与匹配的 CUDA Toolkit。视频导出可使用系统 FFmpeg 或安装的 `imageio-ffmpeg`。

当前节点为 RTX 4090、驱动 595.71.05，系统 CUDA Toolkit 12.8，已有编译器、FFmpeg 和 NVIDIA Vulkan ICD。`nvidia-smi` 顶部 CUDA 13.2 表示驱动支持上限；本环境 PyTorch 使用 CUDA 12.8。cuRobo 使用 `cuda.core` 运行时编译路径。

## 分项验证

```bash
conda activate loom-env
python scripts/check_env.py
python scripts/check_env.py --curobo
```

默认检查依赖一致性、数据读写和 CUDA 矩阵运算；`--curobo` 额外运行官方单次／批量正向运动学与梯度示例。结果及日志写入 `.cache/checks/`，成功的仿真检查会额外保存 `simulation-rgb.png`。

使用 Isaac Sim 前需接受 [NVIDIA Omniverse EULA](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html)。同意后执行：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
# 仅以 root 运行的容器需要此选项。
export OMNI_KIT_ALLOW_ROOT=1
python scripts/check_env.py --sim
```

`--sim` 在独立子进程中启动 Lab，创建本地几何体，执行 GPU PhysX 步进并读取 64×64 RGB 相机。成功条件包含方块落地高度和有效图像，无需下载机器人资产。headless 渲染仍依赖 GPU 图形驱动。该检查不代表机器人任务、完整轨迹采集或状态恢复已通过验收。

## 本机安装与验证记录

验证日期：2026-09-08。环境路径：

```text
/inspire/hdd/global_user/czxs253130598/cache/conda/envs/loom-env
```

实装 Python 3.12.14、Isaac Sim 6.0.1.0、Lab 标签 `v3.0.0-beta2.patch1`、PyTorch 2.11.0+cu128、cuRobo 0.8.0、Warp 1.13.0、cuda-core 1.2.0。Lab 核心／PhysX／Omniverse／资产／可视化子包版本分别为 6.1.14／1.1.3／0.4.2／0.3.4／0.1.0。

| 检查 | 实际结果 |
| --- | --- |
| 依赖一致性 | `pip check` 通过，无冲突 |
| 数据读写 | NumPy、YAML、HDF5、OpenCV、PNG 写入通过 |
| CUDA | RTX 4090 上矩阵运算与 CPU 参考值一致 |
| cuRobo | 单次和 1000 组批量 FK、反向传播运行通过，输出及梯度有限 |
| PhysX | GPU 上执行 120 步，方块从 0.5 m 下落后中心高度为 0.0500 m |
| RTX 相机 | 得到 64×64×3 的有效 RGB 图像，已保存并查看 |
| 脚本检查 | Ruff、Python 编译检查、Bash 语法检查通过 |

Sim 验证在当前 595.71.05 驱动上通过。首次启动包含 RTX 着色器编译，仿真检查共耗时约 217 秒。实际运行命令如下；其中 Vulkan 路径用于本节点选择 NVIDIA ICD：

```bash
OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1 \
VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json \
VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json \
python scripts/check_env.py --curobo --sim
```

本机验证报告位于 `.cache/checks/report.json`，日志为 `pip-check.log`、`curobo.log`、`simulation.log`，图像为 `simulation-rgb.png`。这些运行产物不提交到仓库；依赖版本和验证步骤已保存在上述配置与文档中。

### 直接 pip 安装验证

2026-09-08，在现有环境中对完整仿真命令执行 `pip install --dry-run`，隔离构建的元数据准备与依赖解析均通过；计划仅重新安装本项目和五个 Lab editable 包，无依赖版本切换。验证进程设置 `PIP_CONFIG_FILE=/dev/null`，排除本机旧 pip 索引配置。

现有环境的 `pip check`、数据读写、CUDA 和 cuRobo 检查通过，Ruff 与文档命令语法检查通过。安装解析日志和报告位于 `.cache/setup/direct-pip*`，运行报告位于 `.cache/checks/direct-pip/report.json`。
