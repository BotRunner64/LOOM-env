# 运行

## 安装（一次性）

```bash
conda create -n loom-env python=3.12 pip
conda activate loom-env
python -m pip install uv
uv pip install -e '.[dev]' \
    --extra-index-url https://pypi.nvidia.com \
    --extra-index-url https://download.pytorch.org/whl/cu128 \
    --index-strategy unsafe-best-match
export OMNI_KIT_ACCEPT_EULA=YES
```

两个下载源和 `unsafe-best-match` 都不能省：PyTorch 源也提供 `idna`、`jinja2` 等普通依赖，会遮挡 PyPI 上 Isaac Sim 需要的版本。cuRobo 固定在 `pyproject.toml` 的提交上，因为发布的 v0.8.0 缺少小网格碰撞修复。Lab 用 NVIDIA 源的官方 wheel，不需要克隆源码。

环境自检：`python scripts/check_env.py --curobo --sim`

## 日常流程

```bash
python scripts/prepare_assets.py scene --source-root .cache/assets/source-links
python scripts/prepare_assets.py all                      # 机器人
python scripts/inspect_data.py config configs/collection/<case>.yaml
python scripts/collect.py --collection configs/collection/<case>.yaml --episode-id <id>
python scripts/replay_episode.py outputs/<dir>/episodes/<id> --output-dir <dir>
```

资产准备会写缓存，不能与使用这些缓存的仿真并行。案例见 `ls configs/collection/`；每个案例做什么、判据是什么，在它引用的 task／scene／deployment 三个 YAML 里。脚本用途看 `scripts/` 下各文件的第一行，参数看 `--help`。

## 约定

- 每次采集用新的 episode ID，同 ID 不覆盖；未提交的回合留在 `.incomplete/`
- T 个动作对应 T+1 帧观测与视频：`obs[k] → action[k] → obs[k+1]`
- RGB 不进 HDF5，按相机编码为 MP4；动作与状态保持原数值精度
- 轨迹采集在沙箱外执行

## 已知上游警告（不要重新调查）

启动与运行日志中会反复出现以下信息，均来自上游组件，不影响正确性。不要通过降低日志级别或过滤 stderr 掩盖：

- `grpc/health/v1/health.proto` 重复注册 — Sim 的 `omni.grpc.lib` 与 `omni.datastore` 都编入了同一协议；NVIDIA 的 Protobuf 补丁允许报错后继续
- `Ill-formed SdfPath <>` — PhysX 加载实例化几何时，Fabric/USDRT 把空字符串送入 USD 路径解析器；与项目资产无关
- 无法打开 display、GLFW 初始化失败 — 无桌面节点上官方启动链仍尝试初始化窗口组件
- `omni.hydra` 重复注册、`pxr.Semantics` 弃用 — 上游扩展初始化

`Error` 表示对应组件确实出错，`Warning` 也要按来源解释。日志可能在退出时才批量出现，按时间戳判断阶段；Kit 的完整日志在 `isaacsim/kit/logs/Kit/IsaacLab/3.0/`。
