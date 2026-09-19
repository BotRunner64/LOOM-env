# 资产接入

物体与机器人的来源、准备和接入标准。准备命令见[运行](running.md#日常流程)；版本、哈希与来源 URL 单处维护在 [`assets/catalog.py`](../src/loom_env/assets/catalog.py)（物体）和 [`embodiments/assets.py`](../src/loom_env/embodiments/assets.py)（机器人），本文不复述。

## 核心约束

**物理属性以 USD 为唯一来源。** 物体的质量、碰撞和物理材质由 USD 定义；导入、准备和运行都不依赖独立 metadata，也不在加载时补值或随机化。准备后的 `asset.usda` 是运行物理定义的唯一来源：动态对象有显式质量，每个碰撞体绑定显式静摩擦、动摩擦和恢复系数。

「自包含」指不需要 JSON 或加载器补齐物理属性，**不表示**把全部纹理塞进一个文本文件——USD 可以相对引用同目录的源 USDZ 和纹理，整个资产目录可移动。缺少必要物理属性的模型不猜测补值，列为待处理，通过检查后才接入任务。需要改物理值就直接改 USD 定义并更新固定哈希，不新增 Python 属性表、JSON sidecar 或加载时覆盖。

**规划几何与物理几何分开。** 导出碰撞源网格供 cuRobo 使用，不反向修改 PhysX 的源碰撞配置。源网格与 PhysX 烹饪后的凸分解／SDF 不是同一种表示，规划接触裕量仍需实际执行验证。

## 质量门槛

资产能打开只是准备的起点。新增物体要通过以下检查：

1. 固定源文件与哈希，检查材质依赖、坐标轴、单位和实际网格尺寸。
2. 明确静态／动态行为、质量和碰撞方式。容器不能用填平内腔的单一凸包。
3. 只补当前任务需要的功能标注，并对照网格测量。物体包围盒不能直接充当容器内腔。
4. 跑 `check_scene_assets.py`，检查真实物理静置和释放入容器，查看输出的相机图像。
5. 跑对应任务与物理重放，再用于采集。保留失败尝试，不只保存成功样本。

每份缓存保存来源信息和完整文件哈希；加载时检查文件齐全且与清单一致。准备版本变化或源输入改变后旧缓存被拒绝，需重新准备。

## 物体

源几何来自 [RoboDojo](https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo)（筛选后安装 LOOM USD 定义的物体库）、ManiSkill（桌面 GLB）和 Isaac Sim 6.0 官方物体。仓库只保存小型 USDA 定义（`configs/assets/`）、源几何校验值、来源与固定版本；网格、纹理和转换产物都在 Git 忽略的 `.cache/assets/`。

`.cache/assets/robodojo/` 是**任务库**，不代表上游全部资产；完整上游快照单独存放在 `.cache/assets/robodojo-source/`（约 38.4 GiB，731 个 `object.usdz`），保留原始文件、不覆盖已接入任务的物理定义。上游 metadata 只作归档，不作运行时物理参数来源。下载见 `download_isaacsim_assets.py` 和 `copy_robodojo_assets.py` 的 `--help`；依赖完整不等于效果验收完成。

关节物体是候选库中风险最高的一类：11 款候选均至少有一个刚体缺少显式正质量，全部保持 `pending`。其中 `laptop/00000` 另建 `fixed.usda`，固化源求解器质量并固定底座，作为 `robodojo:laptop_fixed` 接入开盖任务；其余候选的关节拓扑、驱动和材质完整性仍需逐项审查，不因为「能打开」就标为可用。

## 机器人本体

Panda 使用 Isaac Lab 官方 USD（`FRANKA_PANDA_CFG`，HIGH_PD 预设提供驱动增益、力上限和求解迭代配置）；其余五类通过 `assets/convert.py` 调用 Isaac Lab 的 URDF 导入器转换。两类都**未按实机标定**：未显式设置的接触属性由源 USD 和 PhysX 默认值决定，不能写成已标定参数。模型来源相同不保证抓持行为相同，因为项目没有统一复制上游仿真代码里的运行时摩擦或接触 patch 设置。

按臂长选择场景布局，避免目标超出可达范围：

| 本体 | 场景 |
| --- | --- |
| Panda | `tabletop.yaml` |
| Piper、UR5＋WSG、xArm6＋Robotiq | `tabletop_compact.yaml` |
| X5、YAM | `tabletop_near.yaml` |

准备时对源模型的处理，写在各模型的 `asset.json` 中：

- **通用**：当前 URDF 导入器会丢失 GLB 外观，准备入口用固定版本 Trimesh 把 GLB 转成带材质的 OBJ。转换后和仿真启动时都检查每个刚体有可见网格，避免只有碰撞几何却看不到本体。
- **UR5＋WSG**：源文件夹爪关节含重复 `<limit>`，保留第一份完整定义；六个机械臂关节范围恢复为官方 UR5 规格的 ±360°。范围统一维护在 deployment 里，准备流程读取后写入派生 URDF。
- **xArm6＋Robotiq**：源 URDF 是开链模型，ManiSkill 另在 SAPIEN 中加四杆闭环约束。本项目用五条理想平行连杆 mimic 关系表达对应角度约束，只驱动主关节。轻型连杆在源 URDF 的 1000 N·m 上限下会数值失稳，采用 1 N·m 夹爪力矩上限和 0.001 kg·m² 关节转动惯量——这些是仿真控制配置，不是实机参数。
- **YAM**：源文件只有外观网格，准备入口用同一网格补碰撞几何——连杆和夹爪壳体用凸包，两指用 PhysX 原生凸分解以保留外部接触面的凹陷。沿用官方平行夹爪 MJCF 的指间排除规则，只过滤 `tip_left`／`tip_right`。

抓取姿态由 [`embodiments/manipulation.py`](../src/loom_env/embodiments/manipulation.py) 的 `PROFILES` 定义，四元数 xyzw、相对安装基座。标定同时约束夹指朝向和左右指顺序——只检查开合轴水平、接触中心正确或任务成功，会漏掉绕抓取轴的 180° 翻转（UR5 曾因此侧翻）。

## 相机

六类部署统一配置 `front`、`left_wrist`、`right_wrist`：640×480 RGB，每个控制步采样一次（默认 20 Hz）。`front` 固定在世界坐标 `(-0.25, 0.0, 1.55)` m，朝向共同操作区。腕相机跟随机械臂、不跟随活动夹指开合。

`parent_frame` 取 `world` 或 `left/<frame>`／`right/<frame>`；`pose` 是相对该坐标系的**光学位姿**：xyz（m）＋ xyzw 四元数，光学约定为 OpenGL（**-Z 朝前、+Y 朝上**）。父 frame 的轴由资产定义，**不能从名称推断为光学轴**——RoboTwin 资产的 `camera` 按 SAPIEN 约定（+X 前、+Y 左、+Z 上）使用，deployment 里的四元数只表达到 OpenGL 的轴转换。

[`embodiments/frames.py`](../src/loom_env/embodiments/frames.py) 在初始化时解析准备后的 URDF：从目标 frame 沿固定关节回溯到实际 articulation 中最近的刚体，依次合成安装变换。缺失 frame、循环链、或因缺失刚体而必须跨过活动关节时都报错，不默认零位或改挂基座。[`embodiments/cameras.py`](../src/loom_env/embodiments/cameras.py) 渲染前用实测连杆世界位姿合成相机位姿。全局相机焦距 22 mm、水平孔径 24 mm（水平视场约 57°），腕相机 16 mm（约 74°），近裁剪 1 cm；实际内参写入 Episode 元数据。

引用上游安装系不代表已完成实机安装标定。资产自带的相机／支架外观随资产保留，不额外增加外壳、质量或碰撞体。

## 扩展

- **新增物体**：在 `assets/catalog.py` 增加固定来源和所需标注，准备并走完质量门槛。只有新导入格式确有需要时才扩展 `assets/prepare.py`，不为每个模型建独立加载器。
- **新增场景**：新建 `configs/scenes/*.yaml`，引用资产 ID，配置工作区和实例局部位姿／采样范围。场景采样只处理支撑、范围、分离和坐标变换，不依赖任务 ID 或角色名。
- **新增机器人**：在 `embodiments/assets.py` 的 `MODELS` 增加来源，准备资产，在 `configs/deployments/` 增加部署。可达性必须用专家执行验证，不能从臂长推断。

当前工作区限定为水平平面，采样只覆盖受支撑且彼此分离的实体物体。固定 `target_region` 可与物体在 XY 上重叠，使用户能进入目标，但仍必须位于工作区内。目标区域的可见几何、碰撞与任务边界来自同一源定义：有碰撞的餐垫导出同尺寸规划网格，无碰撞的标线导出空规划网格。
