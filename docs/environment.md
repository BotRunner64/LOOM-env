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

## 创建与安装

在项目根目录执行：

```bash
conda env create -f environment.yml
conda activate loom-env
bash scripts/install_deps.sh
```

若 shell 尚未初始化 Conda，先执行 `source "$(conda info --base)/etc/profile.d/conda.sh"`。当前机器已创建该环境时，从激活命令开始即可。

`environment.yml` 创建 Conda 基础环境（pip >=25.1）；安装脚本检查 Python 版本，获取固定 Lab 标签、应用兼容补丁，从 `pyproject.toml` 读取构建依赖和 PyTorch 版本，再以 editable 方式安装本项目的 `sim` extra 与 `dev` 依赖组，最后执行 `pip check`。安装中断后可重跑。已有 `.deps/IsaacLab` 必须匹配指定提交，脚本不会替换其他版本的源码。

Conda 使用 `conda-forge` 并排除默认渠道。安装脚本仅对当前进程使用官方 PyPI、PyTorch 和 NVIDIA 索引，不修改全局 pip 配置。环境设置 `PYTHONNOUSERSITE=1`，避免用户目录下其他项目的包干扰。

Lab 从本项目 `.deps/IsaacLab` editable 安装核心、资产配置、PhysX、Omniverse 与可视化接口。请保留该目录；它不提交进仓库。未安装 Lab 的额外训练框架、遥操作套件或任务集合。Isaac Sim 自身依赖中包含的 Newton 等包会正常安装，运行检查显式选择 PhysX。

## 依赖声明、环境与锁定

`pyproject.toml` 是项目元数据、直接依赖和开发工具配置的统一入口，采用 [PyPA 标准项目配置](https://packaging.python.org/en/latest/guides/writing-pyproject-toml/)。它声明项目需要什么；Conda 管理基础环境，版本快照记录一次经过验证的完整安装结果。

| 位置 | 职责 |
| --- | --- |
| `pyproject.toml` 的 `[project]` | 包名、版本、Python 范围，配置／数值／HDF5／图像读写等核心依赖 |
| `[project.optional-dependencies].sim` | PyTorch、Sim、Lab 子包、Warp 和 cuRobo；用于完整仿真环境 |
| `[dependency-groups].dev` | pytest 和 Ruff，使用 `pip install --group dev` 安装 |
| `[build-system]` / `[dependency-groups].bootstrap` | 本项目构建后端、Lab 与 cuRobo 源码构建所需工具 |
| `[tool.ruff]` | Python 版本、格式与检查范围 |
| `environment.yml` | Conda 的 Python、pip 和环境变量 |
| `requirements/isaaclab.txt` | 已检出的 Lab editable 路径，不重复维护依赖版本 |
| `requirements/pip-linux-64.lock` | 全部 Python 包的版本快照，包含本项目与 Lab 的可移植 editable 路径 |
| `requirements/conda-linux-64.lock` | Python 和 Conda 基础包的精确构建记录 |
| `scripts/install_deps.sh` / `patches/` | 包索引选择、Lab 固定提交与安装前兼容补丁 |

直接依赖在 `pyproject.toml` 中维护，原来的 `base.txt`、`simulation.txt`、`dev.txt` 和 `constraints.txt` 已移除。变更依赖后运行安装与相关检查，再更新完整版本快照。`.lock` 文件当前采用 pip requirements / Conda explicit 格式，不是 uv 或 Poetry 的锁定格式。

仅做离线数据处理或开发基础模块时，在 Python 3.12 环境中执行：

```bash
python -m pip install -e .
# 需要开发工具时添加 dev 依赖组；要求 pip >=25.1。
python -m pip install --group dev
```

默认安装只包含核心依赖，`import loom_env` 不加载 Isaac Sim、cuRobo 或启动仿真。源码位于 `src/loom_env/`，当前只有最小包入口，任务框架仍未实现。

完整仿真使用上节的安装脚本，它会为 `sim` extra 提供正确的 CUDA wheel 索引与打过补丁的 Lab 源码；普通包元数据不负责检出源码和应用补丁。

所选 Lab 标签要求 `coverage==7.6.1`，而发布的 Sim 6.0.1 kernel wheel 要求 `coverage==7.4.4`。补丁只将 Lab 对测试覆盖率工具的约束放宽为 `>=7.4.4,<8`，最终由 Sim 固定到 7.4.4。补丁在安装前应用到源码；不改仿真逻辑，也不直接修改已安装包的元数据。安装保留正常依赖解析并要求 `pip check` 通过。升级 Lab 后需重新评估、移除或更新此补丁。

Lab 仓库根部开发环境预设仍固定 PyTorch 2.10；本项目按发布的 Sim 6.0.1 wheel 要求使用 2.11，并满足 Lab 核心的 `torch>=2.10`。安装按所需子包进行，避免引入不适用的根部开发预设。

需要按本次实际版本完整重建时，在新的环境中执行：

```bash
conda create --name loom-env --file requirements/conda-linux-64.lock
conda env config vars set --name loom-env PYTHONNOUSERSITE=1 PIP_USER=0
conda activate loom-env
bash scripts/install_deps.sh --locked
```

锁定清单也通过正常依赖解析安装；Lab 的固定提交与兼容补丁仍由安装脚本处理。锁定文件针对 Linux x86_64 和 Python 3.12，不能直接用于其他平台。

本项目不依赖参考项目的 editable 路径或 Policy 服务代码。模型训练或服务需要其他依赖时使用独立环境，通过数据文件或 Policy 接口连接。

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
| 完整锁定清单 | `pip install --dry-run` 解析通过，无额外版本切换 |
| 脚本检查 | Ruff、Python 编译检查、Bash 语法检查通过 |

Sim 验证在当前 595.71.05 驱动上通过。首次启动包含 RTX 着色器编译，仿真检查共耗时约 217 秒。实际运行命令如下；其中 Vulkan 路径用于本节点选择 NVIDIA ICD：

```bash
OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1 \
VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json \
VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json \
python scripts/check_env.py --curobo --sim
```

本机验证报告位于 `.cache/checks/report.json`，日志为 `pip-check.log`、`curobo.log`、`simulation.log`，图像为 `simulation-rgb.png`。这些运行产物不提交到仓库；依赖版本和验证步骤已保存在上述配置与文档中。

### pyproject 迁移验证

2026-09-08 已通过新的安装脚本将本项目以 `loom-env==0.1.0.dev0` editable 安装。迁移前后的已安装包版本对比显示：仅新增本项目，既有仿真与数据依赖版本全部一致。

新流程的 `pip check`、基础数据读写与 CUDA 检查通过；wheel 构建成功，并在不含 torch、Isaac Sim、Isaac Lab、cuRobo 的独立临时环境中完成 wheel 安装和 `import loom_env` 检查。该 wheel 导入检查不加载业务模块；完整数据依赖已在主环境验证。含本项目 `sim` extra 的版本快照也已通过正常依赖解析。Ruff 检查、格式检查和 Bash 语法检查通过。

迁移日志位于 `.cache/setup/pyproject-*.log`，基础运行检查位于 `.cache/checks/pyproject/`。此前的 PhysX／RTX 实测记录保留在上一节。
