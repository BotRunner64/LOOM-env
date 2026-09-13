# 任务验证记录

[返回项目入口](../README.md) · [运行指南](implementation.md) · [当前本体抓放验收](embodiments.md#抓放录制)

本文保留历史任务测量、失败尝试和复现命令，供排查与审计。记录只证明指定版本、初态和配置下的结果；早期“待验证”描述反映当时状态，当前支持范围以本体文档为准。`outputs/` 产物未进入 Git，新检出需自行复现或取得对应数据。

## 2026-09-11 任务判据与本体适配


2026-09-11，移除任务成功判定的速度限制后，用当前 `lift_object` 定义逐帧复核已录制的 `outputs/asset-extension/episodes/source-lift-001` 物理状态：第 114 步（5.70 秒）判定成功，物体最低点离桌约 12.7 cm，右手持续夹持。复核报告位于 `outputs/task-criteria/lift-reassessment.json`，包含源记录哈希、原结果及本次使用的任务定义。该结果属于已有轨迹的离线重新判定，未重跑 PhysX；原始轨迹及其 `timeout` 结果保持不变。

同日，在 RTX 4090、驱动 595.71.05 上完成两条新物理采集，均使用右臂、seed 0，采用上述无任务速度限制的判据：

| 任务与记录 | 结果 | 末尾连续 5 个判定步（累计 0.25 秒）的物理证据 |
| --- | --- | --- |
| `lift-right-seed0-r2` | success，114 步，5.70 秒 | 最低角点离桌从 10.18 cm 增至 12.75 cm，持续有右手夹持证据 |
| `place-right-seed0` | success，224 步，11.20 秒 | 全部角点位于动态篮子的实时内部区域，两臂均无夹持证据；末帧距区域边界的最小余量约 0.137 mm，未计入 0.5 mm 容差 |

原始数据位于 `outputs/task-acceptance/episodes/`。两条记录的数据完整性校验通过，前视、左腕、右腕分别保存 115／225 个有效帧。用记录的位姿和接触逐步复核，首次成功步与物理运行的结果一致；报告 [validation.json](../outputs/task-acceptance/validation.json) 保存任务定义、源记录哈希和测量值。视频为原始相机帧的前视／右腕拼接，20 fps：

- [抓起视频](../outputs/task-acceptance/videos/lift-right-seed0-r2.mp4)，[成功帧](../outputs/task-acceptance/videos/lift-right-seed0-r2-final.png)。
- [放入视频](../outputs/task-acceptance/videos/place-right-seed0.mp4)，[成功帧](../outputs/task-acceptance/videos/place-right-seed0-final.png)。

Runner 在满足任务条件时立即结束，所以抓起视频止于抬升阶段，放入视频止于松爪阶段；没有额外录制退回动作或成功后的长时间保持。

复现时，工作目录为仓库根目录，先按 [环境安装](environment.md) 激活 `loom-env` 并完成[场景资产准备](implementation.md#资产准备)。本节点的 NVIDIA Vulkan ICD 路径如下；其他节点应使用其实际路径。输出目录及 episode ID 必须未被占用：

```bash
conda activate loom-env
export OMNI_KIT_ACCEPT_EULA=YES
export OMNI_KIT_ALLOW_ROOT=1
export VK_DRIVER_FILES=/etc/vulkan/icd.d/nvidia_icd.json
export VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json
python scripts/collect.py --collection configs/collection/lift.yaml \
  --output-dir outputs/task-acceptance-repeat --episode-id lift-right-seed0 --seed 0
python scripts/collect.py --collection configs/collection/pick_place.yaml \
  --output-dir outputs/task-acceptance-repeat --episode-id place-right-seed0 --seed 0
python scripts/inspect_data.py episode \
  outputs/task-acceptance-repeat/episodes/place-right-seed0
python scripts/export_video.py outputs/task-acceptance-repeat/episodes/place-right-seed0 \
  outputs/task-acceptance-repeat/videos/place-right-seed0.mp4 --camera front right_wrist
```

预期采集日志输出 `RESULT`，两任务的 `outcome.code` 为 `success`；`inspect_data.py` 应通过完整性检查，导出视频应显示实测抓取／释放结果。首轮验证范围为两个任务各一条右臂、seed 0 轨迹。

随后完成 seed 1、2 的多布局基线。每个任务每个 seed 运行一次，无重试，四条轨迹均成功；数据完整性检查和首次成功步复核通过：

| seed | 物体初始 xy（世界坐标，m，约值） | 抓起 | 放入 |
| --- | --- | --- | --- |
| 1 | (0.4056, -0.0535) | 115 步，5.75 秒 | 208 步，10.40 秒 |
| 2 | (0.3931, -0.0991) | 115 步，5.75 秒 | 223 步，11.15 秒 |

报告位于 [expansion-baseline/validation.json](../outputs/expansion-baseline/validation.json)，记录实际初态、原记录哈希和逐步复核结果。seed 2 的[抓起视频](../outputs/expansion-baseline/videos/panda-lift-seed2.mp4)和[放入视频](../outputs/expansion-baseline/videos/panda-place-seed2.mp4)包含前视、右腕画面。

本轮变化仅为现有采样范围内的目标 xy，篮子配置位置固定；结合 seed 0，覆盖 Panda 右臂的两个任务、三个采样布局，共六条成功轨迹。相同 seed 对应相同采样输入，物理稳定化后的状态仍可能存在小差异，报告保留其测量值；需要严格相同 Query 初态的对照实验应复用记录的初态快照。容器位置／朝向变化、左臂、新本体的任务执行和新标准下的物理重放仍待验证，当前样本不代表整体成功率。

复现本轮四条采集时，沿用上面的环境与资产前置条件，在仓库根目录运行（输出目录未占用）：

```bash
python scripts/collect.py --collection configs/collection/lift.yaml \
  --output-dir outputs/expansion-repeat --episode-id panda-lift-layout --seed 1 --episodes 2
python scripts/collect.py --collection configs/collection/pick_place.yaml \
  --output-dir outputs/expansion-repeat --episode-id panda-place-layout --seed 1 --episodes 2
```

`--episodes 2` 为 ID 添加 `-0000`、`-0001`，分别使用 seed 1、2。每条完成后输出独立 `RESULT`；完整性检查和视频导出沿用上文入口。

第二种本体已选定 Piper。`embodiments/manipulation.py` 集中维护本体的测量 TCP、两指接触连杆、TCP 到抓取区域的偏移、相对基座的抓取姿态以及规划模型。共用专家只组合任务目标与本体变换；夹爪命令和实测开合量使用部署中的限位及仿射关节映射。Panda 沿用原抓取姿态，Piper 使用倾斜 30°、沿 `link6` 局部 +Z 的 0.14 m 抓取区域。抓取姿态还必须保持正常的手掌朝向：`link6` 的 +Y 两指顺序与基座 +Y 同向，相当于基座系绕 Y 转 150°；不因两指接触对称而允许手掌翻转。早期反手版本只通过了任务几何判定，缺少手掌朝向约束，已替换；正常朝向版本的物理验收记录见下文。

Piper 规划模型使用现有缓存 URDF 和固定哈希的 RoboTwin 碰撞球／自碰撞配置，显式转换为 cuRobo 0.8 字段，并从部署生成一致的关节顺序及初始值。抓取物体的球体附着只作用于规划器。规划中的夹指按最大张开位置固定，采用源几何的近似表示，物理执行仍需验收。

预检查发现两项规划表示问题并单独记录：源 `base_link` 球以安装原点为中心、含缓冲半径为 24 mm，与桌面形成固定安装接触；仅在活动臂规划模型中排除该固定连杆，另一臂完整模型仍作为障碍。源右指的首个球（Y=-31 mm，半径 10 mm）完全位于指网格之外（网格 Y 最小约 -4.968 mm），因此移除该虚假障碍，保留其余右指球。源 USD／URDF 的物理碰撞与力限值未修改。对照结果见 `outputs/piper-adaptation/collision-probe-001.log`；规划事件记录配置哈希和固定安装接触连杆，Episode 记录完整抓取参数。

Piper 的导入 USD 使用嵌套刚体层级。生成机器人时逐刚体调用 Isaac Lab 的接触报告 API，传感器路径从实际 USD 中按连杆名称解析，避免默认遍历在外层刚体处停止、内层夹指没有接触报告。这个步骤由所有本体共用的机器人生成入口完成。

物理适配试验保存在 `outputs/piper-adaptation/`，失败尝试也保留。首条完整轨迹 `piper-lift-seed0-002` 使用 0.15 m 偏移：两指各约 10 N 接触，但开始抬升后滑脱，物体回到桌面；这是实际抓持失败，不能算作通过。采用 0.14 m 偏移的 `piper-lift-seed0-003` 已物理成功：144 步、7.20 s，末尾连续 5 帧有右手双指接触且最低点离桌超过 10 cm。视频为 `outputs/piper-adaptation/videos/piper-lift-seed0-003.mp4`。

`piper-place-seed0-001` 在抬升后被专家中遗漏的关节速度条件阻塞：末端距目标约 0.28 mm，最后 2 s 各坐标变化不足 0.1 微米，报告关节速度却约为 0.197 rad/s。该速度不代表所观察到的持续位移；原记录保留为失败证据。现在已删除专家、初始化及运动预览的速度门槛，统一按对应的位置／接触条件检查。重跑 `piper-place-seed0-002` 已在第 158 步进入搬运阶段，确认此问题已解除。

搬运预检查随后暴露原固定路点的问题：释放目标上方额外 10 cm 的搬运目标，在当时的 Piper 抓取姿态下连空场景 IK 也失败。现在搬运高度取实测抓取点高度与越过篮口所需高度的较高者，避免先抬升后再无依据地额外升高；整条路径继续检查另一臂、桌面、篮子及抓持物体的碰撞。`outputs/piper-adaptation/transfer-path-001.json` 记录了从真实抬升末态生成的搬运和下降规划，`piper-place-seed0-003` 已物理完成搬运和释放，末尾物体在篮内但保持计时只有 4 帧；原固定上撤 12 cm 的路点规划失败，中断了继续计时。撤离现改为返回下降前实测到达的 TCP 位姿，重新做完整避碰规划；预检查见 `outputs/piper-adaptation/retreat-path-001.json`。

早期反手版本 `piper-place-seed0-004` 满足任务几何成功条件：235 步、11.75 s，积木完整包围盒在篮子功能区域内且双手均未抓持，连续达到 0.25 s。视频为 `outputs/piper-adaptation/videos/piper-place-seed0-004.mp4`；两项任务的文件完整性和逐帧条件复核保存于 `outputs/piper-adaptation/validation.json`。同一版本还通过 Panda 原始桌面布局、右臂、seed 0 的物理回归：`panda-place-regression-seed0`，208 步、10.40 s，视频位于同目录；共享专家的改动保留了原本体的任务能力。

正常朝向的接近、下降、抬升规划预检查均通过，报告为 `outputs/piper-adaptation/normal-grasp-preflight-001.json`；只修改抓取姿态，不改变本体初始关节、物理资产和任务成功条件。正常朝向版本 `piper-place-normal-seed0-001` 已物理完成抓取、抬升、搬运和释放：211 步、10.55 s，满足原有篮内几何与释放保持条件。完整视频为 `outputs/piper-adaptation/videos/piper-place-normal-seed0-001.mp4`，朝向与任务条件复核为 `outputs/piper-adaptation/normal-grasp-validation.json`。接近时实测末端转动从约 179.94° 降至 47.79°，整条轨迹的两指有向轴与正常方向保持同向。`outputs/piper-adaptation/videos/piper-grasp-orientation-comparison.mp4` 并列展示两个版本前 5 秒的真实固定相机画面，左侧旧版、右侧正常朝向。

Piper 验收范围为右臂、紧凑布局 seed 0；这不代表所有安装位姿、物体、布局或另一侧机械臂已经通过。

复现 Piper 紧凑布局：按[环境文档](environment.md)激活环境，完成[场景资产准备](implementation.md#资产准备)及 `python scripts/prepare_assets.py piper`，从仓库根目录执行（沿用上述 EULA／Vulkan 环境变量）：

```bash
python scripts/collect.py --collection configs/collection/pick_place.yaml \
  --deployment configs/deployments/dual_piper.yaml \
  --scene configs/scenes/tabletop_compact.yaml \
  --output-dir outputs/piper-repeat --episode-id piper-place-seed0 --seed 0
python scripts/export_video.py \
  outputs/piper-repeat/episodes/piper-place-seed0 \
  outputs/piper-repeat/videos/piper-place-seed0.mp4 --camera front right_wrist
python scripts/inspect_data.py episode outputs/piper-repeat/episodes/piper-place-seed0
```

切换 `--collection configs/collection/lift.yaml` 即抓起任务；更换 `--deployment`、`--scene`、`--seed` 分别控制本体、布局配置和采样。每次使用新的 episode ID。成功时终端输出 `RESULT` 的 `outcome.code=success`；失败时同一记录保留阶段事件、物理状态及图像，可用相同视频入口检查。

已有 Episode 保存生成时的任务定义；物理重放应使用对应代码版本，按新标准重新判定需显式选用当前任务定义并另存结果。当前接触测量与专家已接入六类本体，逐类抓放验收与复现入口见[本体文档](embodiments.md#抓放录制)；关节物体、完整厨房、多物体连续整理、异构并行和 Context 配对尚未实现。

此前程序化方块场景的历史数据保留在本地输出中，其物理重放应使用生成时的代码版本；当前入口已迁移到真实资产配置，不提供旧场景适配。

## 场景静置诊断

运行时按 PhysX 的 TGS 提示启用 `enable_external_forces_every_iteration=True`，在每次求解迭代施加外力。保持原始 USDZ 的对照实验中，积木静置角速度噪声由约 0.145 rad/s 降到 0.016 rad/s，静置高度变化约 2 微米；这是用于改善数值表现的求解器设置，源碰撞和动态属性保持不变。该设置随 Episode 记录。
