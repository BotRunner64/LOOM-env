# 本体支持与资产准备

当前共用本体接口支持两台固定基座机械臂组成的双臂部署。任务和数据始终使用 `left`、`right`；关节位置动作保留本体的真实维度，夹爪命令与物理关节分别描述。

| 部署 | 每臂机械臂关节 | 夹爪 | 双臂动作维度 | 末端测量坐标系 |
| --- | --- | --- | --- | --- |
| `configs/deployments/dual_panda.yaml` | 7 | 两个移动关节，宽度命令 0–0.08 m | 16 | `panda_hand` |
| `configs/deployments/dual_piper.yaml` | 6 | `joint7`、`joint8`，宽度命令 0–0.08 m | 14 | `link6` |

这些末端坐标系是模型中的刚体坐标系；Piper 的 `link6` 原点不是指尖抓取中心。抓取专家还需定义抓取位姿和对应偏移。本轮支持关节控制、状态读取、稳定重置、真实物理运动记录及 FK 一致性检查；完整抓取放置任务和碰撞感知运动规划尚未实现。

## 资产来源与存放

- Panda 使用 NVIDIA 官方 Isaac 5.1 Panda USD，沿用已验证的资产版本。
- Piper 使用 [RoboTwin 官方本体包](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0/blob/main/embodiments.zip) 中的 `embodiments/piper/piper.urdf`、视觉网格和碰撞网格。固定版本、下载 URL、SHA256 和转换选项统一维护在 [`embodiments/assets.py`](../src/loom_env/embodiments/assets.py)。
- Piper 限位、惯性和关节坐标取自这个固定的 RoboTwin 模型，尚未对齐某台实机或特定固件版本。RoboTwin 的旧 cuRobo 规划配置仅作参考，不能直接视为当前 cuRobo 版本已验证的规划配置。

下载包、原始模型和生成的 USD 全部保存在 Git 忽略的 `.cache/assets/` 中，也可以通过 `--asset-root` 使用仓库外的共享目录。资产源文件不会复制进 `src/` 或 `configs/`。Piper 只从约 220 MB 的本体包中提取自身目录，不下载物体、背景和轨迹数据集。

```text
.cache/assets/
├── robotwin-<revision>/
│   ├── embodiments.zip
│   └── embodiments/piper/       # 上游源模型及网格
└── piper/
    ├── asset.json              # 来源、转换版本及文件校验和
    └── usd/piper/              # 转换结果及相对路径引用的 payloads
```

## 准备并运行

先按[环境文档](environment.md)准备 `loom-env` 环境及 `.deps/IsaacLab`。在已接受 NVIDIA EULA 的环境中，从仓库根目录执行：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export OMNI_KIT_ALLOW_ROOT=1
export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json
export PYTHONNOUSERSITE=1

python scripts/prepare_piper.py

python scripts/preview_motion.py \
  --deployment configs/deployments/dual_piper.yaml \
  --output-dir outputs/piper \
  --episode-id piper-example-001

python scripts/export_video.py \
  outputs/piper/episodes/piper-example-001 \
  outputs/piper/piper-example-001.mp4 \
  --label 'Dual Piper motion test (no grasp)'

python scripts/check_kinematics.py \
  outputs/piper/episodes/piper-example-001 \
  --output outputs/piper/piper-example-001-fk.json
```

再次录制需使用新的 episode ID，已有 episode 不会覆盖。切换 Panda 时只需把 `--deployment` 改为 `configs/deployments/dual_panda.yaml`。使用自定义资产目录时，准备、预览和 FK 检查三个入口需传入同一个 `--asset-root`。

准备入口首先校验资产包，随后调用 Isaac Lab 自带的 `convert_urdf.py`，固定基座并合并固定关节。每次转换使用新的临时输出目录，成功后替换生成物，避免导入器自动增加 `piper_1/` 后运行时仍读取旧文件。运行前会核验 URDF 与所有生成文件的校验和，缺失或修改过的缓存会明确报错。

## 共用接口与验收

`DualArmArticulation` 提供 `initialize()`、`reset(command)`、`submit(command)`、`observe()`、`write_data_to_sim()` 和 `update(dt)`；调用方拥有物理时钟。它校验实际关节顺序、限位、夹爪映射、末端刚体、固定基座和碰撞几何。两臂命令在写入前一起校验，避免半条非法动作改变一侧控制目标。

重置明确写入关节位置、零速度和控制目标。预览要求机械臂误差低于 0.003 rad、速度低于 0.01 rad/s，夹指误差低于 0.0005 m、速度低于 0.01 m/s，并连续保持 0.25 秒；最长等待 5 秒。它还会先让两臂运动，再重置复验。稳定化过程不进入 episode 时间。

运动诊断使用高刚度 PD 并禁用机器人各连杆的重力。Piper 导入后的刚体层级是嵌套的，当前 Isaac Lab 默认的属性遍历会在第一个刚体停止，因此适配器明确将同一份刚体配置应用到所有连杆。自碰撞保持启用。

12 秒诊断要求每个机械臂关节均移动超过 0.08 rad，保持臂和最终位置误差均低于 0.04 rad，夹爪实际行程超过配置范围的 80%。在 20 Hz 下保存 240 个动作和 241 帧观测。结果、控制参数、重置检查及资产来源写入报告或 episode，诊断失败返回非零退出码。

`check_kinematics.py` 用每一帧实测关节位置计算 cuRobo FK，处理 cuRobo 的 wxyz 与 LOOM/当前 Isaac Lab 的 xyzw 四元数约定，并应用各臂的安装变换。位置误差阈值为 0.1 mm，姿态误差阈值为 0.001 rad。Piper 的检查仅加载 URDF 运动学，不代表已完成碰撞规划、双臂协同或抓取验收。

## 本次验证结果（2026-09-09）

在当前 Isaac Sim／Isaac Lab／cuRobo 环境中，两种本体均通过 12 秒物理运动诊断、两次稳定重置及全部 241 帧的 FK 检查。

| 部署／episode ID | 保持臂最大误差 | 最终关节最大误差 | FK 最大位置差 | 夹爪行程 |
| --- | --- | --- | --- | --- |
| Piper `dual-piper-motion-004` | 0.00519 rad | 0.0000050 rad | 0.00045 mm | 99.3% |
| Panda `dual-panda-motion-002` | 0.02128 rad | 0.0000453 rad | 0.00075 mm | 99.3% |

FK 结果表示同一模型在两个引擎中的数值一致性，不是实机定位精度。Piper 视频位于 `outputs/piper/dual-piper-motion-004.mp4`（960 × 640，20 fps，241 帧）；对应 episode、运动报告和 `kinematics-004.json` 位于同目录下。Panda 回归产物位于 `outputs/preview/`。产物均被 Git 忽略。离线自动化测试共 59 项通过，Ruff 检查与格式检查通过。
