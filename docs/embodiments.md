# 本体支持与资产准备

任务和数据始终使用 `left`、`right`；每臂保留真实的 6／7 个机械臂关节，夹爪命令和物理关节分别描述。通过部署 YAML 切换本体，控制、重置、记录、视频导出及运动学检查使用同一套入口。

| 部署文件（`configs/deployments/`） | 每臂关节 | 夹爪物理关节 | 夹爪命令 | 双臂动作维度 | 末端测量坐标系 |
| --- | --- | --- | --- | --- | --- |
| `dual_panda.yaml` | 7 | 2 个移动关节 | 宽度 0–0.08 m | 16 | `panda_hand` |
| `dual_yam.yaml` | 6 | 2 个移动关节 | 开合坐标 0–0.0939 m | 14 | `gripper` |
| `dual_piper.yaml` | 6 | 2 个移动关节 | 宽度 0–0.08 m | 14 | `link6` |
| `dual_x5.yaml` | 6 | 2 个移动关节 | 宽度 0–0.088 m | 14 | `link6` |
| `dual_ur5_wsg.yaml` | 6 | 2 个移动关节 | 宽度 0.0054–0.11 m | 14 | `wrist_3_link` |
| `dual_xarm6_robotiq.yaml` | 6 | 6 个旋转关节 | 张开角坐标 0–0.81 rad | 14 | `link6` |

当前支持固定安装双臂的关节控制、状态读取、稳定重置、物理运动记录、机械臂 FK 和夹爪耦合一致性检查。末端测量使用表中的刚体坐标系，不等同于指尖抓取中心；六类本体已接入共用抓取放置专家和碰撞感知规划，实际成功回合与复现入口见下文“抓放录制”；实机标定尚未实现。

## 抓放录制

在仓库根目录，激活本文所述 `loom-env` 环境并配置 EULA／Vulkan 环境变量；先准备机器人和场景资产。新接入模型还需要 CUDA 下生成规划碰撞球：

```bash
python scripts/prepare_planning.py yam x5 ur5_wsg xarm6_robotiq
python scripts/collect.py --deployment configs/deployments/dual_x5.yaml \
  --scene configs/scenes/tabletop_near.yaml --arm right \
  --output-dir outputs/x5-pp-repeat --episode-id x5-pp --seed 0
python scripts/export_video.py outputs/x5-pp-repeat/episodes/x5-pp \
  outputs/x5-pp-repeat/views.mp4 --camera front left_wrist right_wrist
```

`prepare_planning.py` 调用已安装 cuRobo 的 `RobotBuilder`，从 URDF 碰撞网格拟合球体；产物和拟合误差指标保存在 `.cache/assets/<model>/planning.json`。当前拟合器忽略 mesh 的 `scale`，因此脚本先在同目录的 `planning/` 中烘焙非单位缩放，再拟合；仿真 URDF 不变。URDF 改变时缓存被拒绝，需要重新生成。自碰撞排除只来自固定合并刚体、相邻刚体及已有物理排除组；不自动忽略初态碰撞或随机采样中未出现的碰撞。固定安装部位只在活动臂规划中豁免，另一臂及场景仍是障碍物。mimic 夹爪只锁定独立关节，保持规划与实际联动关系一致。

抓取点和手掌姿态集中在 `embodiments/manipulation.py`，依据各自夹爪接触几何确定。接触判定使用实测双指力和归一化开口比例，适用于米制开口和 Robotiq 角度命令。动作限位允许 float32 传输的舍入误差，不裁剪动作。夹持、抬升和释放仍由实际物理反馈决定，规划的物体包络不向仿真添加固定约束。

按臂长选择场景布局，避免目标超出可达范围：

| 本体 | 场景配置 |
| --- | --- |
| Panda | `tabletop.yaml` |
| Piper、UR5＋WSG、xArm6＋Robotiq | `tabletop_compact.yaml` |
| X5、YAM | `tabletop_near.yaml` |

## 资产来源与存放

固定提交、下载包 SHA256、转换选项及模型入口统一维护在 [`embodiments/assets.py`](../src/loom_env/embodiments/assets.py)。

- Panda：NVIDIA 官方 Isaac 5.1 Panda USD，沿用已验证的资产版本。
- Piper、ARX-X5、UR5＋WSG：[RoboTwin 官方本体包](https://huggingface.co/datasets/TianxingChen/RoboTwin2.0)。X5 采用包中的 `ARX-X5/X5A.urdf`，不是 RoboDojo 的另一个 X5 v2 模型。
- xArm6＋Robotiq：[ManiSkill-XArm6 官方模型库](https://github.com/haosulab/ManiSkill-XArm6)。
- YAM：[I2RT 官方模型库](https://github.com/i2rt-robotics/i2rt)，MIT 许可，固定 v1.3.5 对应提交 `5b72c47239bd056d0fa6c1a39edeb0537c89443c`。使用 `i2rt/robot_models/arm/yam/v1/yam.urdf` 及其随附平行夹爪；当前接入标准版 YAM v1。独立的 `gripper/linear_4310` MJCF 与该 URDF 的指尖模型、关节坐标不同，未混用两者。

源模型的限位、惯性和关节坐标用于仿真，尚未对齐特定实机或固件。X5 模型的前五个关节包含 ±10 rad 的宽限位；诊断轨迹只在初态附近运动。夹爪命令表示模型中的开合坐标，Robotiq 的旋转坐标不能直接当作米制指尖距离。

下载包、网格、纹理、生成 URDF 和 USD 全部保存在 Git 忽略的 `.cache/assets/` 中，也可通过 `--asset-root` 使用仓库外的共享目录。仓库只保存源码、配置及准备说明。

```text
.cache/assets/
├── robotwin-<revision>/        # 下载包和按需提取的 RoboTwin 模型
├── xarm6-<revision>/           # 官方固定提交源码包
├── i2rt-<revision>/            # YAM 官方固定提交源码包及 MIT 许可
└── <model>/                   # 如 x5、ur5_wsg、yam
    ├── <model>.urdf            # 为仿真准备的模型；上游原件保持不变
    ├── meshes/                 # GLB 转换出的 OBJ、材质和贴图
    ├── asset.json              # 来源、修正记录、版本与文件校验和
    └── usd/<model>/            # USD 及相对路径 payloads
```

## 准备并运行

先按[环境文档](environment.md)安装完整仿真依赖。在已接受 NVIDIA EULA 的环境中，从仓库根目录执行：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export OMNI_KIT_ALLOW_ROOT=1
export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json
export PYTHONNOUSERSITE=1

# 可指定一种或多种。
python scripts/prepare_assets.py x5 ur5_wsg xarm6_robotiq yam
# 全部本地 URDF 本体，包括 Piper 和 YAM；Panda 使用官方 USD，无需转换。
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

运动预览还需先按[场景资产准备](implementation.md#资产准备)生成真实桌面资产。切换机器人只需更换 `--deployment`。再次录制使用新的 episode ID，已有 episode 不覆盖。自定义资产目录时，准备、预览和运动学检查三个入口传入同一个 `--asset-root`。单独准备 Piper 使用 `python scripts/prepare_assets.py piper`。YAM 使用 `python scripts/prepare_assets.py yam`，预览时传入 `--deployment configs/deployments/dual_yam.yaml`。

准备入口先验证下载包，再处理模型并通过 `loom_env.assets.convert` 调用已安装 Isaac Lab 的 `UrdfConverter` API。每次转换使用空的临时目录，成功后替换对应模型的生成物。运行前校验上游 URDF、准备后的 URDF、导出网格和全部 USD 生成文件；旧版本缓存需重新准备。移动缓存目录后也应重新准备，因为中间 URDF 使用本地网格绝对路径。

## 模型处理与共用接口

源文件保持不变，必要处理写入每个模型的 `asset.json`：

- 当前 URDF 导入器会丢失 GLB 外观。准备入口用固定版本 Trimesh 将 GLB 转成带材质的 OBJ，保留子网格及节点变换；转换后及仿真启动时都检查每个刚体是否具有可见网格，避免只有碰撞几何却看不到本体。
- UR5 源文件的夹爪关节含重复 `<limit>`；保留第一份完整定义，避免不同解析器得到不同结果。左右指使用 `[-0.5, +0.5]` 的宽度映射。
- Robotiq 源 URDF 是开链模型，ManiSkill 另在 SAPIEN 中添加四杆闭环约束。本实现用五条理想平行连杆 mimic 关系表达对应角度约束，仅驱动主关节；从动关节的 PD 为零。其轻型连杆在源 URDF 的 1000 N·m 上限下会发生数值失稳，诊断采用 1 N·m 夹爪力矩上限和 0.001 kg·m² 关节转动惯量，参数属于仿真控制配置。沿用 ManiSkill 对夹爪内部及相邻腕部的碰撞排除，保留与外部物体及其他机械臂的碰撞。抓持验收以完整回合的双指接触、抬升和释放结果为准，见下文“抓放录制”。
- YAM 保留官方 URDF 的关节轴、惯性、位置限位和随附夹爪。源文件只有外观网格，准备入口使用相同网格及局部变换补充碰撞几何：连杆和夹爪壳体使用凸包，两指使用 PhysX 原生凸分解，保留外部接触面的凹陷。为避免指间内部接触与抖动，沿用官方平行夹爪 MJCF 的指间排除规则，仅过滤 `tip_left`／`tip_right`，保留它们与物体、其他连杆和另一臂的碰撞。两个关节范围均为约 `[-0.04695, 0]` m。指尖几何检查表明 `q=0` 时两指尖接近闭合，`q=-0.04695` 时张开，因此命令映射为 `q7 = q8 = -width / 2`。`width` 表示模型开合坐标，尚未标定为实机指尖距离。末端测量使用 `gripper` 刚体坐标系。诊断沿用该 URDF 的 1 N·m 机械臂力矩、1 N 夹爪力和 1 rad/s／m/s 速度上限；这些导出模型中的数值不代表已校验的实机驱动参数。

`DualArmArticulation` 提供 `initialize()`、`reset(command)`、`submit(command)`、`observe()`、`write_data_to_sim()` 和 `update(dt)`；调用方拥有物理时钟。启动时校验关节顺序、限位、夹爪映射、末端刚体、固定基座和碰撞几何。两臂命令在写入前一起校验。

机械臂及夹爪的力／力矩上限默认取自 URDF，Robotiq 采用上文说明的保守夹爪上限。诊断把机械臂速度上限限制为源限值与 3 rad/s 的较小值，夹爪限制为源限值与 1 m/s 或 rad/s 的较小值。PD 增益按模型配置；机器人全部连杆禁用重力，自碰撞启用。导入器可能嵌套刚体，因此共用适配器明确将刚体配置应用到每根连杆。

## 三路相机

六类部署统一配置 `front`、`left_wrist`、`right_wrist`，均为 640×480 RGB、每个控制步采样一次（默认 20 Hz）。抓取采集与运动预览读取同一部署配置，不再由预览脚本单独覆盖相机。

- Panda 的 `front` 固定在机器人侧 `(-0.25, 0.0, 1.55)` m，朝向共同操作区 `(0.45, 0.0, 0.80)` m；其余五类运动诊断仍使用外部观察位置 `(2.0, 0.0, 1.9)` m。
- 腕相机通过固定安装变换跟随机械臂，不跟随活动夹指开合。资产已有相机安装系时直接引用；没有安装系时，由 deployment 显式定义仿真安装位姿。

| 本体 | deployment 引用的父坐标系（每侧） | 安装来源 |
| --- | --- | --- |
| Piper | `camera` | URDF 的 `link6 → camera_mount → camera` |
| X5 | `camera` | URDF 的 `link6 → camera_base → camera` |
| UR5＋WSG | `camera` | URDF 的 `wrist_3_link → camera_base → camera` |
| xArm6＋Robotiq | `camera_link` | URDF 的 `link6 → camera_link`，加下述官方光学偏移 |
| Panda | `panda_hand` | deployment 显式安装变换 |
| YAM | `gripper` | deployment 显式安装变换 |

`parent_frame` 取 `world` 或 `left/<frame>`／`right/<frame>`；`pose` 是相对该坐标系的**光学位姿**：xyz（m）加 xyzw 四元数，光学约定为 OpenGL（-Z 朝前、+Y 朝上）。父 frame 的轴由资产定义，不能从名称推断为光学轴。RoboTwin 三类资产的 `camera` 按 SAPIEN 的 +X 前、+Y 左、+Z 上使用，deployment 的零平移和 `[0.5, -0.5, -0.5, 0.5]` 四元数只表达 OpenGL 光学轴转换，支架位置和倾角全部取自 URDF。[RoboTwin 位姿同步实现](https://github.com/RoboTwin-Platform/RoboTwin/blob/9633927e61a47df88db3806b5aade5c9488c79fd/envs/_base_task.py#L385-L399)。

xArm 的 `camera_link` 本身并非光学中心。[ManiSkill 固定版本配置](https://github.com/haosulab/ManiSkill/blob/b860fb7fc8ce086fb5f27f554f9c7146650f1cee/mani_skill/agents/robots/xarm6/xarm6_robotiq.py#L407-L423) 在该 frame 上使用 SAPIEN 位姿 `p=[0, 0, -0.05]` m、`q(wxyz)=[√0.5, 0, √0.5, 0]`。转换到本项目的 OpenGL／xyzw 后为 `pose=[0, 0, -0.05, 0, 0, -√0.5, √0.5]`。分辨率、焦距和裁剪范围采用下述 LOOM 配置，与上游成像内参不完全相同。

[`embodiments/frames.py`](../src/loom_env/embodiments/frames.py) 在初始化时解析已验证的准备后 URDF：从目标 frame 沿固定关节回溯到实际 articulation 中最近的刚体，依次合成安装变换，再应用 deployment 的光学位姿。导入时可继续合并固定关节，无需为安装 frame 新增物理刚体。直接引用现存刚体时不读取 URDF，Panda 的原生 USD 走此路径；当前不解析原生 USD 中任意被合并的固定 frame。缺失 frame、循环链、缺失刚体而必须跨过活动关节时均报错，不默认为零位或改挂基座。挂载计算不使用连杆质心或外观 mesh 的局部原点。

[`embodiments/cameras.py`](../src/loom_env/embodiments/cameras.py) 渲染前用实测连杆世界位姿与解析出的光学安装位姿合成相机位姿，并同步全部腕相机；USD 光学 prim 放在世界系，避免当前 Fabric 对嵌套子相机的陈旧变换。采集与运动预览共用此路径。Episode 的 `sampled_parameters.camera_mounts` 记录原始父 frame、实际刚体和相对刚体的光学位姿，作为派生诊断信息；运行配置的安装来源仍为 URDF 与 deployment。图像及位姿在同一采样时刻缓存。每次初帧取图前预热渲染，不推进物理或控制时间。任务环境在创建时先完成一次 GPU articulation 步进初始化，之后再完整恢复 Episode 初态，避免冷启动重放的机器人外观仍停留在加载姿态。当前采集与诊断均为单环境。

`focal_length` 与 `horizontal_aperture` 使用 mm，`clipping_range` 使用 m。全局相机焦距 22 mm、水平孔径 24 mm（水平视场约 57°），腕相机焦距 16 mm（约 74°），近裁剪距离 1 cm。实际内参写入 Episode 元数据；采样时相机世界位姿保存在真值 `cameras/<name>/pose_world`，与该相机 RGB 时间戳对应，不作为模型观测。

目前模拟光学传感器；资产自带的相机／支架外观随资产保留，不额外增加外壳、支架质量或碰撞体。Panda、YAM 的显式安装是仿真方案；引用上游安装系也不表示已经完成实机安装标定。导出指定视角：

```bash
python scripts/export_video.py outputs/pick_place/episodes/panda-place \
  outputs/pick_place/panda-left-wrist.mp4 --camera left_wrist
```

### 检查相机挂载

只修改相机挂载时无需重新转换 USD。未准备资产时，先按本文资产准备及[场景资产准备](implementation.md#资产准备)完成机器人、桌面和物体的准备。按[环境文档](environment.md)激活 `loom-env` 并设置其中的 EULA／Vulkan 环境变量，在仓库根目录执行：

```bash
# 为每次验证选一个新的目录，已有 Episode 不覆盖。
run_dir=outputs/camera-mounts-repeat
for robot in piper x5 ur5_wsg xarm6_robotiq panda yam; do
  python scripts/preview_motion.py \
    --deployment "configs/deployments/dual_${robot}.yaml" \
    --output-dir "$run_dir/$robot" --episode-id "$robot-mount"
  python scripts/check_kinematics.py \
    "$run_dir/$robot/episodes/$robot-mount" \
    --output "$run_dir/$robot/kinematics.json"
  python scripts/export_video.py \
    "$run_dir/$robot/episodes/$robot-mount" "$run_dir/$robot/motion.mp4" \
    --camera front left_wrist right_wrist
done
```

预期每类产生 12 秒、241 帧的三路录制，运动诊断和 `kinematics.json` 中的 `passed` 均为 `true`。查看三视角视频，检查相机随腕运动、夹指开合不带动相机、图像朝向与各安装定义一致，以及是否存在遮挡。FK 数值通过只验证坐标一致性，不能替代这些画面检查。`Mount frame not found` 时检查 deployment 的 frame 拼写与对应准备后 URDF；`untracked moving joint` 表示实际刚体集合缺失了活动关节的子连杆，应调查导入结果，不应把该关节当成固定关节。

Piper 完整抓放视野检查使用紧凑布局和右臂：

```bash
python scripts/collect.py \
  --deployment configs/deployments/dual_piper.yaml \
  --scene configs/scenes/tabletop_compact.yaml --arm right \
  --output-dir outputs/camera-mounts-grasp-repeat \
  --episode-id piper-place-mount --seed 0
python scripts/export_video.py \
  outputs/camera-mounts-grasp-repeat/episodes/piper-place-mount \
  outputs/camera-mounts-grasp-repeat/views.mp4 --camera front left_wrist right_wrist
```

X5、YAM 的运动诊断初始姿态不面向桌面操作区，腕相机视野需要结合任务姿态检查。xArm 按官方光学旋转呈现画面，夹指出现在画面侧缘。

## 诊断检查

重置明确写入关节位置、零速度和控制目标。预览要求机械臂位置误差低于 0.003 rad；移动夹爪位置误差低于 0.0005 m，旋转夹爪低于 0.003 rad，连续到位 0.25 秒，最长等待 5 秒。关节及夹爪速度保留为诊断数据，不作为重置验收条件。还会先运动再重置复验，稳定化过程不进入 episode 时间。

12 秒运动诊断要求每个机械臂关节均移动超过 0.08 rad，保持臂和最终位置误差均低于 0.04 rad，夹爪实际命令行程超过配置范围的 80%。20 Hz 下保存 240 个动作与 241 帧观测。任何诊断失败都会保存结果并返回非零退出码。

`check_kinematics.py` 用每帧实测关节位置计算 cuRobo FK，处理四元数约定和安装变换后，与仿真测量的末端位姿比较，阈值为 0.1 mm／0.001 rad。相机检查独立通过 cuRobo 计算 deployment 所引用 frame 的 FK，再合成光学偏移，与渲染器记录的相机世界位姿比较；不调用运行时的固定链解析器。按每路相机的采样时间选择对应关节状态，并检查采样周期和时间戳一致，世界固定相机也检查外参。同时核对夹爪各物理关节是否满足声明的耦合关系，允许 0.0005 m 或 0.003 rad 的误差。夹爪耦合阈值用于空载诊断，带物抓持时应结合接触状态解释偏差。FK 检查衡量两个引擎对同一模型的数值一致性，不代表实机定位精度。
