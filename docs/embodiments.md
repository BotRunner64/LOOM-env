# 本体支持与资产准备

任务和数据始终使用 `left`、`right`；每臂保留真实的 6／7 个机械臂关节，夹爪命令和物理关节分别描述。通过部署 YAML 切换本体，控制、重置、记录、视频导出及运动学检查使用同一套入口。

| 部署文件（`configs/deployments/`） | 每臂关节 | 夹爪物理关节 | 夹爪命令 | 双臂动作维度 | 末端测量坐标系 |
| --- | --- | --- | --- | --- | --- |
| `dual_panda.yaml` | 7 | 2 个移动关节 | 宽度 0–0.08 m | 16 | `panda_hand` |
| `dual_piper.yaml` | 6 | 2 个移动关节 | 宽度 0–0.08 m | 14 | `link6` |
| `dual_x5.yaml` | 6 | 2 个移动关节 | 宽度 0–0.088 m | 14 | `link6` |
| `dual_ur5_wsg.yaml` | 6 | 2 个移动关节 | 宽度 0.0054–0.11 m | 14 | `wrist_3_link` |
| `dual_xarm6_robotiq.yaml` | 6 | 6 个旋转关节 | 张开角坐标 0–0.81 rad | 14 | `link6` |
| `dual_openarm.yaml` | 7 | 2 个移动关节 | 左 0.016–0.088 m，右 0–0.088 m | 16 | `openarm_left_link7`／`openarm_right_link7` |

当前支持固定安装双臂的关节控制、状态读取、稳定重置、物理运动记录、机械臂 FK 和夹爪耦合一致性检查。末端测量使用表中的刚体坐标系，不等同于指尖抓取中心；完整抓取放置、碰撞感知规划和实机标定尚未实现。

## 资产来源与存放

固定提交、下载包 SHA256、转换选项及模型入口统一维护在 [`embodiments/assets.py`](../src/loom_env/embodiments/assets.py)。

- Panda：NVIDIA 官方 Isaac 5.1 Panda USD，沿用已验证的资产版本。
- Piper、ARX-X5、UR5＋WSG：[RoboTwin 官方本体包](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0)。X5 采用包中的 `ARX-X5/X5A.urdf`，不是 RoboDojo 的另一个 X5 v2 模型。
- xArm6＋Robotiq：[ManiSkill-XArm6 官方模型库](https://github.com/haosulab/ManiSkill-XArm6)。
- OpenArm：[Enactic 官方描述库](https://github.com/enactic/openarm_description) 中的 v1 模型。分别提取左右臂，以独立固定基座安装到桌面；保留两侧不同的关节限位，不包含原模型的躯干。

源模型的限位、惯性和关节坐标用于仿真，尚未对齐特定实机或固件。X5 模型的前五个关节包含 ±10 rad 的宽限位；诊断轨迹只在初态附近运动。夹爪命令表示模型中的开合坐标，Robotiq 的旋转坐标不能直接当作米制指尖距离。

下载包、网格、纹理、生成 URDF 和 USD 全部保存在 Git 忽略的 `.cache/assets/` 中，也可通过 `--asset-root` 使用仓库外的共享目录。仓库只保存源码、配置及准备说明。

```text
.cache/assets/
├── robotwin-<revision>/        # 下载包和按需提取的 RoboTwin 模型
├── xarm6-<revision>/           # 官方固定提交源码包
├── openarm-<revision>/         # 官方固定提交源码包
└── <model>/                   # 如 x5、ur5_wsg、openarm_left
    ├── <model>.urdf            # 为仿真准备的模型；上游原件保持不变
    ├── meshes/                 # GLB 转换出的 OBJ、材质和贴图
    ├── asset.json              # 来源、修正记录、版本与文件校验和
    └── usd/<model>/            # USD 及相对路径 payloads
```

## 准备并运行

先按[环境文档](environment.md)准备 `loom-env` 环境及 `.deps/IsaacLab`。在已接受 NVIDIA EULA 的环境中，从仓库根目录执行：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export OMNI_KIT_ALLOW_ROOT=1
export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json
export PYTHONNOUSERSITE=1

# 可指定一种或多种；openarm 同时准备左右臂。
python scripts/prepare_assets.py x5 ur5_wsg xarm6_robotiq openarm
# 全部本地 URDF 本体，包括 Piper；Panda 使用官方 USD，无需转换。
python scripts/prepare_assets.py all

python scripts/preview_motion.py \
  --deployment configs/deployments/dual_x5.yaml \
  --output-dir outputs/x5 \
  --episode-id x5-example-001

python scripts/export_video.py \
  outputs/x5/episodes/x5-example-001 outputs/x5/motion.mp4 \
  --label 'Dual X5 motion test (no grasp)'

python scripts/check_kinematics.py \
  outputs/x5/episodes/x5-example-001 --output outputs/x5/kinematics.json
```

切换机器人只需更换 `--deployment`。再次录制使用新的 episode ID，已有 episode 不覆盖。自定义资产目录时，准备、预览和运动学检查三个入口传入同一个 `--asset-root`。单独准备 Piper 使用 `python scripts/prepare_assets.py piper`。

准备入口先验证下载包，再处理模型并调用 Isaac Lab 自带的 `convert_urdf.py`。每次转换使用空的临时目录，成功后替换对应模型的生成物。运行前校验上游 URDF、准备后的 URDF、导出网格和全部 USD 生成文件；旧版本缓存需重新准备。移动缓存目录后也应重新准备，因为中间 URDF 使用本地网格绝对路径。

## 模型处理与共用接口

源文件保持不变，必要处理写入每个模型的 `asset.json`：

- 当前 URDF 导入器会丢失 GLB 外观。准备入口用固定版本 Trimesh 将 GLB 转成带材质的 OBJ，保留子网格及节点变换；转换后及仿真启动时都检查每个刚体是否具有可见网格，避免只有碰撞几何却看不到本体。
- UR5 源文件的夹爪关节含重复 `<limit>`；保留第一份完整定义，避免不同解析器得到不同结果。左右指使用 `[-0.5, +0.5]` 的宽度映射。
- Robotiq 源 URDF 是开链模型，ManiSkill 另在 SAPIEN 中添加四杆闭环约束。本实现用五条理想平行连杆 mimic 关系表达对应角度约束，仅驱动主关节；从动关节的 PD 为零。其轻型连杆在源 URDF 的 1000 N·m 上限下会发生数值失稳，诊断采用 1 N·m 夹爪力矩上限和 0.001 kg·m² 关节转动惯量，参数属于仿真控制配置。沿用 ManiSkill 对夹爪内部及相邻腕部的碰撞排除，保留与外部物体及其他机械臂的碰撞。夹爪的接触抓持能力还需完整任务验证。
- OpenArm 保留左右臂各自的模型和原生夹爪 mimic，按部署中的安装位姿分别生成。当前凸包碰撞模型在腕部 `link5`／`link7` 间产生内部干涉，接触诊断测得约 3.1 kN 的非预期力；只排除该碰撞对，其余自碰撞及外部碰撞保持启用。当前左爪的指间碰撞在开合坐标约 0.015 m 时发生，左侧命令下限据此设为 0.016 m，右侧保持 0 m。这是当前仿真资产的可达控制范围，不是实机指尖距离标定值。

`DualArmArticulation` 提供 `initialize()`、`reset(command)`、`submit(command)`、`observe()`、`write_data_to_sim()` 和 `update(dt)`；调用方拥有物理时钟。启动时校验关节顺序、限位、夹爪映射、末端刚体、固定基座和碰撞几何。两臂命令在写入前一起校验。

机械臂及夹爪的力／力矩上限默认取自 URDF，Robotiq 采用上文说明的保守夹爪上限。诊断把机械臂速度上限限制为源限值与 3 rad/s 的较小值，夹爪限制为源限值与 1 m/s 或 rad/s 的较小值。PD 增益按模型配置；机器人全部连杆禁用重力，自碰撞启用。导入器可能嵌套刚体，因此共用适配器明确将刚体配置应用到每根连杆。

## 验收范围

重置明确写入关节位置、零速度和控制目标。预览要求机械臂误差低于 0.003 rad、速度低于 0.01 rad/s；移动夹爪误差低于 0.0005 m，旋转夹爪低于 0.003 rad，速度低于各自单位下的 0.01/s，连续稳定 0.25 秒，最长等待 5 秒。还会先运动再重置复验，稳定化过程不进入 episode 时间。

12 秒运动诊断要求每个机械臂关节均移动超过 0.08 rad，保持臂和最终位置误差均低于 0.04 rad，夹爪实际命令行程超过配置范围的 80%。20 Hz 下保存 240 个动作与 241 帧观测。任何诊断失败都会保存结果并返回非零退出码。

`check_kinematics.py` 用每帧实测关节位置计算 cuRobo FK，处理四元数约定和安装变换后，与仿真测量的末端位姿比较，阈值为 0.1 mm／0.001 rad。同时核对夹爪各物理关节是否满足声明的耦合关系，允许 0.0005 m 或 0.003 rad 的误差。这里的微小 FK 差异表示两个引擎对同一模型的数值一致性，不是实机定位精度。

## 本轮验证结果（2026-09-09）

六类本体均完成 12 秒物理运动诊断、运动后重置复验、241 帧 FK／夹爪耦合检查和视频导出，全部通过。下表取左右臂中误差较大或行程较小的一侧；夹爪行程相对于各自声明的控制范围计算。

| 本体 | 保持臂最大误差（rad） | 最终关节误差（rad） | 最大 FK 位置差（mm） | 最小夹爪行程比例 |
| --- | ---: | ---: | ---: | ---: |
| Panda | 0.02128 | 4.52e-05 | 0.00075 | 99.33% |
| Piper | 0.00519 | 4.99e-06 | 0.00045 | 99.34% |
| ARX-X5 | 0.00313 | 8.58e-06 | 0.00041 | 99.35% |
| UR5＋WSG | 0.00367 | 9.54e-06 | 0.00065 | 99.38% |
| xArm6＋Robotiq | 0.00272 | 9.20e-06 | 0.00049 | 99.05% |
| OpenArm | 0.00503 | 7.44e-06 | 0.00044 | 99.34% |

本轮另有 70 项单元测试通过，Ruff 检查及格式检查通过。结果均为上述仿真配置下的运动诊断，不代表完成抓取任务或实机性能验证。

本地验证产物位于 Git 忽略的 `outputs/embodiments/`：

- [四类新本体同屏视频](../outputs/embodiments/other-robots-motion.mp4)：ARX-X5、UR5＋WSG、xArm6＋Robotiq、OpenArm，20 fps、241 帧。
- `<name>/motion-003.mp4`：各本体完整视频。
- `<name>/dual-<name>-final-003-report.json` 和 `<name>/fk-003.json`：物理运动、重置及 FK／夹爪耦合报告。
- `<name>/episodes/dual-<name>-final-003/`：部署快照与 HDF5 轨迹。
- `validation-final.json`、`video-validation.json`：各阶段退出状态与视频解码检查。
