# 开发与仿真环境

项目要求 **Isaac Sim >=6**，不限制小版本；Isaac Lab 按上游兼容关系选择。目标运行平台是 Linux x86_64、NVIDIA RTX GPU，物理后端为 PhysX，图像渲染使用 Isaac RTX。

项目直接依赖只在 `pyproject.toml` 中声明。普通库、PyTorch、Warp 不重复固定精确版本，由 Sim / Lab 的依赖关系决定。当前使用 Python 3.12 创建仿真环境；项目自身只要求 Python >=3.12，Sim 的 wheel 仍有自己的 Python ABI 限制。

## 创建环境

从仓库根目录执行。Conda 只提供 Python，依赖使用 pip 安装：

```bash
conda create -n loom-env python=3.12 'pip>=25.1'
conda activate loom-env
```

已有同名环境时先检查安装来源，避免源码 editable 子包覆盖官方 wheel 的模块。

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

## 资产转换

机器人与场景准备统一通过 `python -m loom_env.assets.convert` 调用已安装 Lab 的转换 API。该模块只负责在独立仿真进程中转换，资产校验和发布仍由 `scripts/prepare_assets.py` 完成。URDF 保留原固定基座、合并固定关节和关节驱动参数；桌面保留三角网格转换设置。

在仓库根目录、激活完整仿真环境并设置上文 EULA / Vulkan 变量后，可用已准备的 Piper URDF 和共享桌面源文件检查转换，不覆盖正式资产：

```bash
python -m loom_env.assets.convert urdf .cache/assets/piper/piper.urdf \
    .cache/checks/conversion/piper-usd --fix-base --merge-joints \
    --joint-stiffness 400 --joint-damping 40 --headless
python -m loom_env.assets.convert mesh \
    /inspire/hdd/global_user/czxs253130598/projects/sim_projects/ManiSkill/mani_skill/utils/scene_builder/table/assets/table.glb \
    .cache/checks/conversion/table.usd --headless
```

首次使用应先按[机器人说明](embodiments.md#准备并运行)准备 Piper，并按[场景资产说明](scene-assets.md)取得桌面源文件；其他节点替换共享路径。成功时输出 `ASSET_CONVERTED`，结果分别为 `piper-usd/piper/piper.usda` 和 `table.usd`。缺少输入文件时直接报错；仿真启动问题按上文排查。

迁移后正式环境的依赖检查、数据读写、CUDA、cuRobo 正向运动学／梯度、小网格碰撞和 GPU PhysX／RTX 检查全部通过，报告为 `.cache/checks/environment-migration/checks/report.json`，渲染图像为同目录 `simulation-rgb.png`。复现：仓库根目录激活 `loom-env` 并设置上文 EULA／Vulkan 变量后，执行 `python scripts/check_env.py --curobo --sim --output-dir .cache/checks/environment-migration/checks`；各项 `passed` 应为 `true`。该检查不替代抓放任务效果验收。
