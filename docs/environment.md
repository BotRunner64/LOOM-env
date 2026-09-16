# 开发与仿真环境

项目要求 **Isaac Sim >=6**，不限制小版本；Isaac Lab 按上游兼容关系选择。目标运行平台是 Linux x86_64、NVIDIA RTX GPU，物理后端为 PhysX，图像渲染使用 Isaac RTX。

项目直接依赖只在 `pyproject.toml` 中声明。普通库、PyTorch、Warp 不重复固定精确版本，由 Sim / Lab 的依赖关系决定。当前使用 Python 3.12 创建仿真环境；项目自身只要求 Python >=3.12，Sim 的 wheel 仍有自己的 Python ABI 限制。

## 创建环境

从仓库根目录执行。Conda 提供 Python 环境，项目依赖使用 `uv pip` 安装：

```bash
conda create -n loom-env python=3.12 pip
conda activate loom-env
python -m pip install uv
```

已有同名环境时先检查安装来源，避免源码 editable 子包覆盖官方 wheel 的模块。

## 安装依赖

仿真依赖默认安装；开发环境额外安装 pytest 和 Ruff：

```bash
uv pip install -e '.[dev]' \
    --extra-index-url https://pypi.nvidia.com \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    --index-strategy unsafe-best-match
pip check
```

`-e` 使源码修改直接生效；`[dev]` 安装开发工具，只运行仿真时改为 `uv pip install -e .`，保留两个下载源参数和 `--index-strategy unsafe-best-match`。开发工具通过 `project.optional-dependencies.dev` 声明。安装时确认 uv 输出的目标环境为已激活的 `loom-env`。

uv 默认只从首个包含某包的源选择版本；PyTorch 源也包含 `idna`、`jinja2` 等普通依赖，但可能没有 Isaac Sim 要求的版本。`unsafe-best-match` 允许综合所有源的候选版本，避免这些包遮挡 PyPI 上的所需版本。该策略以信任列出的下载源为前提，行为接近 pip，见 [uv 多源解析说明](https://docs.astral.sh/uv/pip/compatibility/#packages-that-exist-on-multiple-indexes)。

上述命令使用默认 PyPI 和两个额外源。uv 不读取 `pip.conf`；若本机的 uv 配置更改了默认源且缺包，可追加 `--index-url https://pypi.org/simple`。

[NVIDIA 源](https://pypi.nvidia.com/isaaclab/)提供官方 Isaac Lab 统一 wheel，已包含资产、PhysX、Omniverse 和可视化模块，无需克隆 Lab、安装五个源码子包或修改第三方源码。`isaaclab[isaacsim]>=3.0.0b2` 允许所需 API 系列的 Beta wheel，并由其 `isaacsim` extra 选择配套 Sim；项目额外表达的 Sim 要求只有 `>=6`。这不保证任意 Lab 与任意 Sim 都能混用。

PyTorch 下载源提供 CUDA 12.8 构建，但添加该源并不强制安装此构建；实际版本由依赖解析决定。安装后按下文检查 `torch.version.cuda`，并运行 CUDA 与仿真检查。

cuRobo 从 `pyproject.toml` 指定的上游提交安装。保留这一源码提交是因为已发布的 v0.8.0 缺少小网格碰撞查询修复；该提交包含上游修复，不再应用本地补丁。后续有包含修复的正式发行版时可切换到发行包。

## 运行与验证

宿主机需要可供容器访问的 NVIDIA RTX GPU、图形驱动和 Vulkan ICD，以及 GLIBC >=2.35、Vulkan/OpenGL/EGL 运行库。Git 用于获取 cuRobo；资产获取和源码扩展可能还需要 git-lfs、C/C++ 工具链及匹配的 CUDA Toolkit。视频导出使用 `imageio-ffmpeg`。

从仓库根目录、激活环境后执行：

```bash
python scripts/check_env.py --curobo
```

检查项目依赖范围、`pip check`、数据读写、CUDA 矩阵运算、cuRobo 正向运动学及梯度，以及小网格的远处无碰撞和近处碰撞。结果写入 `.cache/checks/report.json`，子进程日志在同目录。版本检查通过不能替代仿真和实际任务验收。

首次启动若出现 [NVIDIA Omniverse EULA](https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html) 提示，阅读并按提示确认：

```bash
python scripts/check_env.py --sim
```

`--sim` 创建本地几何体，执行 GPU PhysX 步进，检查方块落地高度及 320×320 RGB 相机输出；无需机器人资产。成功时保存 `.cache/checks/simulation-rgb.png`。headless 渲染仍依赖 GPU 图形驱动。实际任务的资产准备、采集和重放见[运行指南](implementation.md)。

## 当前节点与排查

- 非交互运行：已同意 EULA 时，可设置 `OMNI_KIT_ACCEPT_EULA=YES` 避免交互提示。
- root 容器：仅在以 root 身份运行且启动被拒绝时，设置 `OMNI_KIT_ALLOW_ROOT=1`。

当前节点使用 RTX 4090。若 Vulkan 未选择 NVIDIA ICD，在运行仿真前设置本节点路径：

```bash
export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json
```

该路径属于宿主机配置，不是所有机器的通用要求。`nvidia-smi` 顶部的 CUDA 版本是驱动支持上限；实际 PyTorch CUDA 构建可用 `python -c 'import torch; print(torch.version.cuda)'` 查看。

- 安装找不到 Lab Beta wheel：检查 NVIDIA 下载源是否可访问；仅使用 PyPI 可能得到旧版 Lab。
- uv 报 `idna`／`jinja2` 无匹配版本，且提示包在首个源中找到：确认安装命令包含 `--index-strategy unsafe-best-match`。
- 下载超时或 Git TLS 连接中断：重跑安装命令可复用已完成的缓存；若反复失败，检查网络／代理到报错下载地址的连接。`Resolved ... packages` 只表示依赖解析完成，仍需等待下载和安装结束。
- 依赖冲突：先运行 `pip check`，检查是否混入旧 editable 子包；不要用 `--no-deps` 或修改第三方源码绕过正式安装的依赖解析。
- cuRobo 小网格检查失败：核对是否安装了项目声明的上游提交，不能仅看 `0.8.0` 版本前缀。
- 仿真启动失败：查看 `simulation.log`，检查 GPU、Vulkan ICD 和 EULA 设置；首次 RTX 着色器编译可能较慢。

## 采集日志与已知剩余问题

`Error` 表示对应组件发生错误，不能因为仍有输出文件就视为正常；`Warning` 也需要按来源解释。任务结果、数据完整性和画面质量分别验收。正常使用不应依靠降低日志级别或过滤 stderr 来隐藏问题。

项目规划器只在首次规划时加载碰撞几何，后续使用 cuRobo 的位姿更新和碰撞启停接口；允许接触或附着目标时关闭其场景碰撞，释放后恢复。每次更新仍清空图搜索缓存，避免复用旧场景路径。这消除了项目反复全量加载场景引发的 `Mesh already in cache` 警告。

项目所有仿真入口通过 [`launch_app`](../src/loom_env/runtime/app.py) 启动已安装的官方 Lab experience。启动前只注册有清单的 Lab 扩展与 `.kit` 文件，避免把 `__pycache__`、非扩展源码目录当成扩展；Sim 自身扩展目录继续正常扫描。直接路径注册使用 Kit 的 [`/app/exts/paths`](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/extensions_advanced.html)。

启动入口根据已安装 Replicator 的 MDL 目录生成进程专用材质 TOML，同时设置实际材质搜索路径和允许列表，并启用材质库使用的 `omni.kit.context_menu`。临时配置在进程退出时清理，不修改安装目录或用户全局配置。保持标准材质路径启用。启动参数按独立 argv 项传递，支持带空格的路径，启动完成或失败后恢复调用者参数。

显式将 `/UJITSO/geometry` 和 `/persistent/UJITSO/geometry` 设为 false，关闭实验性几何处理；保留 UJITSO 的纹理、材质和物理缓存服务。这个开关**没有消除下表中的共享流式系统警告**。实际设置随回合保存，不能用配置值代替对渲染状态和画面的检查。该功能的实验性说明见 [NVIDIA UJITSO 文档](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/ujitso.html)。

以下是 2026-09-13 正式环境的处理状态。没有修改依赖源码或过滤日志；明确覆盖相关启动设置并展开 USD diagnostics，避免原生警告被静音。任务与独立模拟检查共用的显式画质设置见[相机渲染](implementation.md#相机渲染)。

| 日志 | 已确认来源、影响与待查内容 |
| --- | --- |
| `grpc/health/v1/health.proto` 重复注册、absl 初始化提示 | 已用独立动态库加载实验定位：Sim 的 `omni.grpc.lib` 与 `omni.datastore` 都注册同一协议，组合加载即复现。NVIDIA 的 Protobuf 补丁允许报错后继续；根因已确认，修复尚未实施。见[调查与最小复现](protobuf-startup.md)。 |
| `extension.toml` 缺失（`__pycache__`、`isaaclab_visualizers`） | 已修复：用直接扩展路径替代 Lab apps/source 的宽泛扫描；不删除缓存或依赖目录。 |
| 无法打开 display、GLFW 初始化失败 | 无桌面的节点上，官方启动链仍尝试初始化窗口组件。本次 headless 采集能继续，但窗口组件的警告仍存在。 |
| `omni.hydra` 重复注册、`pxr.Semantics` 弃用 | 来自上游扩展初始化；项目未直接使用旧 Semantics API，具体上游调用链尚未完整定位。 |
| Replicator 无材质配置、材质库缺少 `omni.kit.context_menu` | 已修复：启动前提供完整 Replicator 材质配置并启用材质库所需的上下文菜单依赖。 |
| Fabric 变换读取与 geometry streaming 同时启用 | 部分处理，警告仍在：已关闭实验性 geometry，但 Sim 6 的共享 streaming active 仍为 true。原生代码读取的 geometryTypes 在空值时仍选择 Volume，警告条件并不只看 geometry 开关。关闭整个 UJITSO 会新增纹理／材质／PhysX cooking 服务警告，因此未采用；不关闭机器人运动所用的 Fabric 变换同步。 |
| DLSS 自动增大输入尺寸（旧配置） | 原默认 DLSS 在 640×480 相机上触发 320×240 内部输入尺寸警告。任务环境和 `smoke_sim.py` 现共用 DLAA＋quality；独立模拟检查同时明确启用每次 TGS 迭代施加外力。 |
| USD diagnostics 被静音 | 已展开为 `Ill-formed SdfPath <>`。原始实例化 Panda 和纯代码实例化方块均在 PhysX 加载触发的 Fabric/USDRT 同步中复现；空场景、篮子、取消实例化的 Panda／方块不复现。无需外部资产即可触发；具体原生字段仍未定位，未实施修复。见[调查与对照](usd-diagnostics.md)。 |
| `failed to fork`、`nvidia-smi` / telemetry 启动失败 | 历史日志中存在；本轮基线采集未复现，单独运行 `nvidia-smi -L` 正常。资源限制或启动链的具体原因尚未确认。 |

日志可能在退出时才批量出现在终端，不能仅按终端行位置判断发生阶段；应看时间戳。终端日志之外，Kit 完整日志位于当前环境的 `isaacsim/kit/logs/Kit/IsaacLab/3.0/`，应同时检查。

复现项目内修复：仓库根目录激活完整 `loom-env` 环境，并按上文完成 EULA、GPU/Vulkan 和资产准备后执行：

```bash
mkdir -p .cache/checks/collection-logs
python -m pytest tests/test_planner_world.py -q
python scripts/collect.py --collection configs/collection/pick_place.yaml \
  --output-dir .cache/checks/collection-logs/check --episode-id place-check --seed 0 \
  > .cache/checks/collection-logs/check.log 2>&1
python scripts/inspect_data.py episode \
  .cache/checks/collection-logs/check/episodes/place-check
```

GPU 回归测试检查障碍物移动、首次允许接触、目标碰撞禁用及恢复；缺少 cuRobo/CUDA 时会跳过，跳过不算通过仿真验收。采集应以状态码 0 退出，`RESULT` 为 `success`、`video_error` 为 `null`，且无 `Mesh already in cache`；其余已列出的上游日志仍可能出现。检查 `check/videos/place-check.mp4` 的抓取、移动、释放画面，以及三个相机视频的内容；检查命令验证视频哈希、尺寸、帧率、解码帧数和轨迹对齐。重复运行需更换 episode ID 或输出目录，不覆盖既有回合。

本轮对照结果保存在 `.cache/checks/collection-logs/`：`baseline.log` 与 `fixed.log` 的重复网格警告分别为 8 和 0 条，两次均在 205 步完成放置、退出状态为 0，三个相机视频及拼接预览生成成功，修复后的轨迹完整性校验通过。动作、关节状态和物体位姿记录一致，接触力最大绝对差约为 `1.9e-6`；完整测量见 `comparison.json`，关键帧见 `comparison.png`，修复后视频见 `fixed/videos/place-fixed.mp4`。关键帧检查覆盖抓取、转移和入篮，不能替代对所有种子、机器人和上游渲染风险的验收。完整测试为 168 项通过（包含真实 GPU 碰撞回归测试）。这些验证产物位于已忽略的缓存目录，不进入 Git。


启动配置修复的复验产物位于 `.cache/checks/startup-fixes/`。与原显式画质基线相比，完整采集中的扩展清单、无材质配置、上下文菜单警告分别从 2、1、1 条降为 0；共享 geometry streaming 警告仍为 1 条。两次均 205 步成功，动作、关节状态与物体位姿一致，仅接触力最大绝对差 `1.9073486328125e-6`。三个相机各 206 帧，视频与轨迹完整性校验通过。`comparison.png` 对照抓取、转移和释放，`cameras.png` 检查正面及腕部视角，`comparison.json` 保存逐项测量；预览为 `collection/videos/place-startup.mp4`。

独立检查通过：方块落地高度 `0.0500 m`，RGB 输出 `(320, 320, 3)`，无最小 DLSS 输入尺寸或外力迭代警告。使用 320×320 是因为 64×64 即使使用 DLAA 仍低于原生最小输入尺寸。无相机桌面网格转换成功；完整回归测试 168 项通过，Ruff 检查通过。本次结果覆盖默认 seed 0 采集及上述入口，不代表所有资产和种子已验收。

复验启动配置：在仓库根目录激活完整 `loom-env` 环境，按上文准备 EULA、GPU/Vulkan 与默认采集资产后运行；输出目录必须使用新的位置：

```bash
mkdir -p .cache/checks/startup-recheck
python scripts/collect.py --seed 0 --episode-id place-recheck \
  --output-dir .cache/checks/startup-recheck/collection \
  > .cache/checks/startup-recheck/collection.log 2>&1
python scripts/inspect_data.py episode \
  .cache/checks/startup-recheck/collection/episodes/place-recheck
LOOM_CHECK_OUTPUT_DIR=.cache/checks/startup-recheck/smoke \
  python scripts/smoke_sim.py > .cache/checks/startup-recheck/smoke.log 2>&1
python -m pytest -q
```

采集应返回 `success`、`video_error: null`，独立检查应输出 `ISAACLAB_PHYSX_RGB_PASS`。查看采集视频及 `smoke/simulation-rgb.png`；查看 manifest 的 `spec.sampled_parameters.rendering.runtime_settings`，其中两个 geometry 设置为 false，Fabric 变换读取和共享 streaming active 仍为 true。后两项和实际剩余警告必须如实保留，不以手工改写状态或过滤日志通过验收。

## 资产转换

机器人与场景准备统一通过 `python -m loom_env.assets.convert` 调用已安装 Lab 的转换 API。该模块只负责在独立仿真进程中转换，资产校验和发布仍由 `scripts/prepare_assets.py` 完成。URDF 保留原固定基座、合并固定关节和关节驱动参数；桌面保留三角网格转换设置。

在仓库根目录、激活完整仿真环境后，可用已准备的 Piper URDF 和共享桌面源文件检查转换，不覆盖正式资产：

```bash
python -m loom_env.assets.convert urdf .cache/assets/piper/piper.urdf \
    .cache/checks/conversion/piper-usd --fix-base --merge-joints \
    --joint-stiffness 400 --joint-damping 40 --headless
python -m loom_env.assets.convert mesh \
    /inspire/hdd/global_user/czxs253130598/projects/sim_projects/ManiSkill/mani_skill/utils/scene_builder/table/assets/table.glb \
    .cache/checks/conversion/table.usd --headless
```

首次使用应先按[机器人说明](embodiments.md#准备并运行)准备 Piper，并按[场景资产说明](scene-assets.md)取得桌面源文件；其他节点替换共享路径。成功时输出 `ASSET_CONVERTED`，结果分别为 `piper-usd/piper/piper.usda` 和 `table.usd`。缺少输入文件时直接报错；仿真启动问题按上文排查。

迁移后正式环境的依赖检查、数据读写、CUDA、cuRobo 正向运动学／梯度、小网格碰撞和 GPU PhysX／RTX 检查全部通过，报告为 `.cache/checks/environment-migration/checks/report.json`，渲染图像为同目录 `simulation-rgb.png`。复现：仓库根目录激活 `loom-env` 后，执行 `python scripts/check_env.py --curobo --sim --output-dir .cache/checks/environment-migration/checks`；各项 `passed` 应为 `true`。该检查不替代抓放任务效果验收。
