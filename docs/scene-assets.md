# 场景资产接入与扩展

首批选用 ManiSkill 木桌、RoboDojo 积木和收纳篮，保留真实模型与材质。资产来源、固定 revision、源文件 SHA-256、单位、尺寸和功能标注集中在 [资产目录](../src/loom_env/assets/catalog.py)，本文不重复维护版本值。

## 已接入资产

| 资产 ID | 用途 | 碰撞与功能 |
| --- | --- | --- |
| `maniskill:table` | 桌面工作区 | 源 GLB 转 USD，桌面顶中心作为原点；桌板为长方体，桌腿保留静态三角网格 |
| `robodojo:tea_carton_pack` | 双 Panda 交接候选 | 源 `Clutter/tissue/00001`，描述为六盒茶饮包装；交接效果仍待验收 |
| `robodojo:juice_carton` | 横放单盒果汁交接变体 | 源 `Rigid/juice_carton/00000`，保留吸管外观和源碰撞；质量 0.12 kg、摩擦 0.3 来自既有 USD；运行结果见[交接变体](implementation.md#交接的少量物体与布局变体) |
| `robodojo:brick` | 动态操作对象 | 保留源动态刚体和凸分解碰撞；补充 Panda 抓取中心 |
| `robodojo:hand_brush` | 持工具扫动 | 源 `Rigid/broom/00000`，刚体刷毛与 22 mm 宽柄部；完整 USD 质量 0.04 kg、摩擦 0.45，见[扫动说明](implementation.md#持工具扫物体入区域) |
| `robodojo:mouse` | 弧面物体扫动变体 | 源 `Rigid/mouse/00004`，约 11.2 × 7.3 × 3.7 cm；完整 USD 质量 0.09 kg、摩擦 0.45 |
| `robodojo:waffle` | 格纹物体扫动变体 | 源 `Rigid/waffle/00000`，约 9.6 × 9.2 × 3.2 cm；完整 USD 质量 0.08 kg、摩擦 0.65，按刚体处理 |
| `robodojo:basket` | 动态目标容器 | 保留源动态刚体和 SDF 碰撞；记录局部放置区域，随实测容器位姿更新 |
| `robodojo:plate` | 推上餐垫的浅盘 | 源凸分解碰撞，约 13 cm 直径；Panda 推动接触高度标注 |
| `robodojo:box` | 推入收纳区的打开纸盒 | 源盒壁／翻盖与碰撞，翻盖不可动；Panda 推动接触高度标注 |
| `loom:placemat` | 固定薄餐垫 | 仓库内小型 USDA，原生长方体碰撞，22 × 22 cm、厚 1 mm |
| `loom:storage_zone` | 可见桌面收纳标线 | 仓库内小型 USDA，无物理／规划碰撞，28 × 38 cm |

源几何保存在本地 `.cache/assets/robodojo/`，LOOM 的小型 USD 定义在 `configs/assets/robodojo/` 固定版本。共享上游目录保持原样。准备入口通过 `--source-root` 指向含 RoboDojo 与 ManiSkill 的目录布局，产物写入被 Git 忽略的 `.cache/assets/scenes/`。
ManiSkill GLB 通过 `loom_env.assets.convert` 调用 Isaac Lab 的标准 MeshConverter，保留外观。该源文件没有物理定义；桌板碰撞使用资产目录中实测包围尺寸的长方体，桌腿及下方横梁保留原三角网格。准备入口必须找到唯一匹配测量范围的桌板组件才会替换。原桌板三角网格与篮子 SDF 的接触导致持续摆动；长方体提供稳定平面。规划碰撞缓存由同一长方体和下方网格生成，与实际桌子碰撞一致。

准备后的 `asset.usda` 是运行物理定义的唯一来源：动态对象具有显式质量，每个碰撞体绑定显式静摩擦、动摩擦和恢复系数。它可以保留同目录源 USDZ／纹理的相对引用；“自包含”指不需要 JSON 或加载器补齐物理属性，不表示把全部纹理塞进一个文本文件。整个资产目录可移动。

所有物体在导入、准备和运行时都以 USD 为物理属性唯一来源。RoboDojo 的 `object.usda` 定义质量和物理材质，相对引用同目录的原始 `object.usdz`，后者保留外观与碰撞几何。仓库只保存小型 USDA 定义和源几何校验值，大型几何包留在缓存。旧物体 metadata 文件及读取逻辑已删除，不再作为准备输入，也没有兼容回退。

本次一次性迁移保留已知源值，将源单一摩擦系数写为静、动摩擦，恢复系数显式设为 0；原有物理材质保留。桌面、浅盘和餐垫沿用已审查的 0.5 摩擦，浅盘沿用改动前测得的求解器质量；这些数值现在直接存于 USD。它们是仿真定义，未声称经过实物称量。惯性由 PhysX 根据明确质量和碰撞形状计算。

`scripts/check_scene_assets.py` 对比加载后的 USD 材质绑定与准备清单，并将求解器 `body_mass_kg` 与 USD 质量核对。准备清单是可再生成的校验报告，不参与补写物理值。旧准备版本被拒绝，必须重新准备。

规划网格与物理资产分开处理：导出碰撞源网格供 cuRobo 使用，不反向修改 PhysX 的源碰撞配置。源网格与 PhysX 烹饪后的凸分解／SDF 不是同一种表示，规划接触裕量和实际执行仍需验证，不能宣称它们完全一致。

## 质量门槛

资产可打开只是准备的起点。新增资产需要通过以下检查：

1. 固定源文件与哈希，检查材质依赖、坐标轴、单位和实际网格尺寸。
2. 明确静态／动态行为、质量和碰撞方式。容器不能使用填平内腔的单一凸包。
3. 仅补当前任务需要的功能标注，并对照网格测量。物体包围盒不能直接充当容器内腔。
4. 执行 `scripts/check_scene_assets.py`，检查真实物理静置和释放入容器，查看输出的相机图像。
5. 执行对应专家任务及物理重放，再把该资产用于采集。保留失败尝试，不能只保存成功样本。

每份缓存保存来源信息和完整文件哈希；加载时检查文件是否齐全、内容是否与缓存清单一致。准备版本变化后旧缓存会被拒绝，必须重新准备；源输入改变时也应重新准备对应资产。每个 Episode 保存实际资产版本、稳定后的物理初态和展开配置。

### 本次迁移的检查入口

在仓库根目录激活 `loom-env`，按[环境说明](environment.md)完成 EULA 与 GPU 配置。本机 `.cache/assets/source-links` 已配置；新机器安装 RoboDojo 库后需建立如下目录关系：

- `source-links/RoboDojo/Assets/Object/RoboDojo` 指向 `.cache/assets/robodojo`（内含完整 `object.usda`）。
- `source-links/ManiSkill` 指向固定版本的 ManiSkill 源码目录（包含桌面 GLB）。

这些是普通目录或符号链接，`--source-root` 可使用任何满足该布局的目录。随后重建所有已接入资产：

```bash
python scripts/prepare_assets.py scene --source-root .cache/assets/source-links
python scripts/check_scene_assets.py --collection configs/collection/handover.yaml \
  --output-dir outputs/asset-physics/tea-check
```

`--source-root` 下的 RoboDojo 目录须包含已安装的 LOOM `object.usda` 和对应 `object.usdz`；原始上游几何目录不能直接替代完整资产。USD 定义和源几何均检查哈希。健康检查预期输出 `passed: true`，茶饮包装 `body_mass_kg` 约为 0.45，每项 `physics_checks` 均通过。图像和 `validation.json` 位于指定输出目录。已有缓存若提示 `Obsolete` 或 `Prepared definition changed`，重跑准备；不要绕过版本检查。

本次删除旧物体标注后的验证产物位于 `outputs/asset-physics/usd-only-{tea,pick-place,box,plate}/`，各目录包含 `validation.json` 和相机图像，覆盖全部 8 个任务资产。交接、抓放和推移的完整专家效果需要基于新资产单独验收。

## 迁移资产

迁移时完整复制 `.cache/assets/`，保留 USD、纹理和清单的目录关系。网格转换入口会将输出 USD 中指向同目录内文件的绝对引用改为相对引用；运行时提供的 `gltf/pbr.mdl` 等材质模块名保持原样。

旧转换产物可能仍包含旧机器的绝对贴图路径，即使文件校验通过，也会在渲染时报告 `References an asset that can not be found`。检查报错中的路径及本地 `scenes/table/textures/`。重新准备资产会使用新的转换逻辑；若直接修复已有 USD，必须先核对原清单并备份，只修改已确认的引用，同时更新 `asset.json` 中对应文件的 SHA-256。修复后需重新启动采集并检查桌面纹理，已运行进程和已录制视频不会自动更新。

## 如何扩展

**新增资产：** 在 `assets/catalog.py` 增加一个固定来源及所需标注，准备并验收缓存。新导入格式确有需要时才扩展 `assets/prepare.py`；不为每个模型创建独立加载器。

**新增场景：** 新建 `configs/scenes/*.yaml`，引用资产 ID，配置工作区和实例局部位姿／采样范围。通过 collection 引用该场景，并选择匹配的部署。场景采样只处理支撑、范围、分离和坐标变换，不依赖任务 ID 或角色名。可达性仍需专家执行验证。

**新增任务：** 增加任务配置与状态判定类，在 `tasks/__init__.py` 显式注册。已有环境输出所有动态物体位姿、速度和两臂接触证据，以及容器区域；任务通过角色绑定读取目标实例。不同动作流程另加专家并在 `runtime/build.py` 注册，Runner 和存储协议无需修改。

当前工作区限定为水平平面，采样只覆盖受支撑且彼此分离的实体物体。固定 `target_region` 可与物体在 XY 上重叠，使物体能进入目标；它仍必须位于工作区内。目标区域的可见几何、碰撞与任务边界来自同一源定义：餐垫碰撞导出为同尺寸规划网格，无碰撞标线导出空规划网格。固定底座笔记本已补齐单铰链状态、重置和任务判据，见[开盖说明](implementation.md#固定底座笔记本开盖)；其他关节类型、堆叠初态、非矩形功能区域仍需按具体任务实现。

## 全库 USD 定义与检查

### 上游原始资产与任务资产的区别

`.cache/assets/robodojo/` 是经过筛选并安装 LOOM USD 定义的物体库，不代表 RoboDojo 上游的全部资产。完整上游快照单独存放在 `.cache/assets/robodojo-source/Assets/`，保留原始文件，不覆盖已接入任务的物理定义。原始 metadata 仅作为上游文件归档，不作为 LOOM 运行时的物理参数来源。

当前固定版本的上游包含 731 个 `object.usdz`：Rigid 466、Clutter 193、Geometry 49、Articulation 11、Garment 6、Fluid 4、Dynamic 2。Articulation 的 11 款分为笔记本电脑 4 款、烤面包机 4 款、弹簧按钮 2 款、蛋盒 1 款；`Geometry/drawer/00000` 不在该类别中；本次直接检查源 USD 确认它只有 1 个刚体、没有 PhysicsJoint，因此不是可开合的关节抽屉。

完整 `Assets/` 约 38.4 GiB，还包括材质、房间、机器人、传感器、布局与轨迹文件。下载不代表物理属性合格或已接入任务。尤其是关节物体，必须另外核对关节、碰撞、质量和材质，并补齐环境中的状态、重置和重放支持。

复现下载：从仓库根目录激活 `loom-env`，确保环境已安装 `huggingface_hub`、可访问 Hugging Face，并预留至少 40 GiB 可用空间。本次原始快照固定到下述 revision；与任务库旧 revision 相比，仅 165 个布局 JSON 发生变化，物体和材质文件相同。旧 revision 的部分布局文件在上游返回 HTTP 403，因此原始归档使用完整可获取的新快照，现有任务版本保持不变：

```bash
python - <<'PY'
from huggingface_hub import snapshot_download

snapshot_download(
    repo_id="RoboDojo-Benchmark/RoboDojo",
    repo_type="dataset",
    revision="91f76c28d93dd20c5fa46ce6a5a1d96a4f384acd",
    allow_patterns=["Assets/**"],
    local_dir=".cache/assets/robodojo-source",
    max_workers=8,
)
PY
```

该标准入口可重复运行以续传。首次本机下载另保存 `source-files.json`、`download-manifest.json` 和 `download.log`，按远端 LFS SHA-256 或 Git blob SHA-1 核验每个文件；查看清单的 `complete` 和 `failures` 判断下载是否完成，不能只看目录存在。网络失败先检查代理及日志，重试时保留已经完成的文件。所有大文件和本机清单均位于 Git 忽略目录。

### 关节物体候选收录

已将全部 11 款关节候选的轻量 USD 入口收录到 [`configs/assets/robodojo/Articulation/`](../configs/assets/robodojo/Articulation/)。每个 `object.usda` 相对引用同目录的 `object.usdz`，并在 `customLayerData` 固定源 revision 和 SHA-256；仅包装入口，不修改源质量、材质、关节或驱动。大型几何和纹理安装在被 Git 忽略的 `.cache/assets/robodojo/Articulation/`，合计约 27 MiB。

| 类型 | 型号目录 | 每款刚体数 | 每款活动关节数 |
| --- | --- | --- | --- |
| `SpringButton` | `00002`、`00003` | 2 | 1 个移动关节 |
| `egg_holder` | `00000` | 2 | 1 个旋转关节 |
| `laptop` | `00000`–`00003` | 2 | 1 个旋转关节 |
| `toaster` | `00000` | 4 | 1 个旋转、2 个移动关节 |
| `toaster` | `00001` | 3 | 1 个旋转、1 个移动关节 |
| `toaster` | `00002` | 6 | 1 个旋转、4 个移动关节 |
| `toaster` | `00004` | 4 | 3 个移动关节 |

以上计数来自 USD 定义，不表示已验证 PhysX 中的有效自由度。11 款均至少有一个刚体缺少显式正质量，全部保持 `pending`；质量为 0 表示没有提供项目要求的显式正质量，不能据此断言源资产在 PhysX 中质量为零。未从外部 metadata 补值，也未修改固定底座设定。关节拓扑、驱动、材质完整性及实际运动仍需后续审查；原始候选保持待处理；其中 `laptop/00000` 已另建 `fixed.usda`，固化源求解器质量并固定底座，作为 `robodojo:laptop_fixed` 接入[开盖任务](implementation.md#固定底座笔记本开盖)。不将原始 11 款全部标为可用。

安装和检查继续使用已有入口。从仓库根目录激活 `loom-env`，先按上文下载源快照，再执行（无需 GPU）：

```bash
python scripts/copy_robodojo_assets.py \
  --source-root .cache/assets/robodojo-source/Assets/Object/RoboDojo
python scripts/check_asset_library.py \
  --output outputs/asset-physics/articulation-import-audit.json
```

安装覆盖全部 470 个本地 RoboDojo 入口（459 个原有物体及 11 个关节候选），相同几何复用，源与目标必须是独立目录。安装预期 `dependency_issues=0 copy_failures=0`；每项 revision、校验值、依赖结果和关节清单保存在 `.cache/assets/robodojo/manifest.json`。Isaac Sim 提供的标准 MDL 模块在报告中单列，不要求复制到模型目录。

候选收录时全库审查结果为 `USD_READY 478 PENDING 11`（另含 Isaac Sim 和已准备任务入口）；总数随任务资产变化；接入固定底座笔记本后为 `USD_READY 479 PENDING 11`，原始候选仍保持待处理。检查命令因存在待处理候选而返回 1，这是预期结果，不代表复制失败。报告的 `articulation` 列出刚体、关节类型、连接对象、轴、限位及发现的物理缺项；旋转限位单位为 USD 的度，移动限位使用 stage 长度单位。检查尚不覆盖完整关节拓扑或驱动有效性，修复质量也不自动成为已验收任务资产。源几何哈希不符时先核对快照版本；缺材质时查看清单中的 `unresolved_dependencies`。

### 单刚体物体库

本地 RoboDojo 的单刚体集合保留 459 个物体，覆盖 `Rigid`、`Geometry` 和 `Clutter`；对应 USD 定义位于 [`configs/assets/robodojo/`](../configs/assets/robodojo/)。版本固定为 `a14409d7fae673c00499e01fd88b4457df6351b1`。复制入口直接按这些定义定位源几何、检查哈希，从 USD 测量尺寸、检查物理属性，不再读取外部属性表。

在仓库根目录激活 `loom-env`，从上游几何目录安装至本地缓存：

```bash
python scripts/copy_robodojo_assets.py --source-root /path/to/RoboDojo/Assets/Object/RoboDojo
python scripts/check_asset_library.py
```

复制入口只复制几何包并安装 USDA；源和目标必须是独立目录。约需 8 GiB 空间，不需要 GPU。相同文件复用，内容冲突报错。`manifest.json` 保存来源、逐文件哈希、依赖检查和物理就绪状态，不给加载器提供物理参数。标准 MDL 模块由 Isaac Sim 提供。

迁移时全库检查覆盖 459 个 RoboDojo 入口、5 个 Isaac Sim 入口和当时的 8 个任务资产，从实际 USD 检查单刚体、质量、碰撞和绑定材质。单刚体可以位于子节点；检查器会验证该刚体的质量和碰撞材质，不因层级不同误判属性缺失。输出 `outputs/asset-physics/library-audit.json`，该次 472 个入口通过、0 个待处理；有待处理项时退出码为 1。定义完整不代表已完成具体场景的物理验收。

本次清理移除了 178 个 RoboDojo 和 10 个 Isaac Sim 候选：缺质量或摩擦定义的模型不猜测补值，缺刚体／碰撞的模型不为扩充库而重建。四个物理定义完整、刚体位于子节点的彩色积木保留，未修改源 USD。移除记录位于 `outputs/asset-physics/library-prune.json`；清理只作用于 LOOM 本地物体库及配置，共享上游资产不变。实际场景准备仍要求目录中所选物体的入口满足对应场景接口，新增子节点刚体模型时需一并验证位姿与重置。
以后新增物体先提供完整 USD。确需修改资产物理值时直接修改对应 USD 定义，更新固定哈希并重新准备；不新增 Python 属性表、JSON 物理 sidecar 或加载时覆盖。全库报告只用于查看与验收，不参与仿真加载。

### RoboDojo 示例宫格

在仓库根目录激活完整环境，按[环境说明](environment.md)配置 EULA、GPU 和 Vulkan 后执行：

```bash
python scripts/render_asset_gallery.py --collection robodojo
```

默认从已复制的模型中渲染配置所列的代表条目，选择在 [`configs/assets/robodojo_preview.json`](../configs/assets/robodojo_preview.json) 中维护。输出位于 `.cache/previews/robodojo-sample/`：`food.jpg` 为食品与饮料，`everyday.jpg` 为玩具与日用品，`tools_containers.jpg` 为工具与容器。每格标注类别／型号和源尺寸；独立缩放取景，不是相同比例尺。单模型 PNG 在 `images/`，源路径、实测尺寸与失败信息在 `render-manifest.json`。可用 `--group food`、`--limit 3` 只检查少量条目，或用 `--output-dir` 保存另一份预览。渲染保留源材质、不推进物理时间，也不修改源资产。

## Isaac Sim 官方物体缓存

保留 5 个物理定义完整的源模型，清单在 [`assets/isaacsim.py`](../src/loom_env/assets/isaacsim.py) 单处维护，下载、检查和宫格预览共用：

- **4 个抓取物体**：蓝、绿、红、黄积木；刚体位于子节点，原始质量、碰撞和材质完整。
- **1 个容器**：小号 KLT 箱。

其他候选入口及仅由它们使用的依赖已清理，共用材质和纹理按保留资产的依赖闭包保留。这些模型尚未注册到项目任务目录，接入时仍按本文质量门槛检查实际抓放效果。

使用与本机 Isaac Sim 6.0.1 对应的 **6.0 资产根目录**，版本关系见 [NVIDIA 资产设置文档](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_faq.html)。在仓库根目录激活完整环境后运行：

```bash
conda activate loom-env
python scripts/download_isaacsim_assets.py
```

默认下载到 `.cache/assets/isaacsim-6.0/`，可用 `--output-dir` 指定缓存、`--workers` 调整下载线程数（默认 8）。下载与依赖检查不启动仿真，不需要 GPU；需要访问 NVIDIA 公开 S3 服务。当前保留模型及依赖共约 195 MiB，建议准备至少 0.5 GB 空间。

- `Isaac/Props/`：保留的源 USD、网格、材质和纹理。
- `download-manifest.json`：源版本地址、入口清单、ETag、大小、更新时间及逐文件 SHA-256。
- `dependency-report.json`：USD 引用、USDZ 包内成员及 MDL 纹理路径的检查结果。

首次运行只获取选定入口，再递归补齐引用。后续运行复用清单并校验文件；重跑会跳过校验通过的文件，未完成的单个文件重新下载。官方版本目录可能更新，复现同一快照需保留并复用下载清单。源文件内容不做修改，Isaac Sim 自带的 `OmniPBR` 由运行环境提供。

退出码 0 表示下载及上述依赖检查通过；1 表示下载失败；2 表示有缺失依赖，查看报告中的 `unresolved`。修改保留清单时应同步更新缓存清单并按依赖闭包清理，不能按文件名删除仍被其他模型引用的纹理。当前保留集合的依赖检查无缺失。依赖完整不代表效果验收完成；下载不会补默认物理属性。

### 渲染物体宫格图

在仓库根目录激活完整 `loom-env` 环境，完成下载，并按[环境文档](environment.md)配置 EULA 和本机 GPU/Vulkan 后运行：

```bash
python scripts/render_asset_gallery.py
```

输出到 `.cache/previews/isaacsim-assets/`：`objects.jpg`、`containers.jpg` 为分类宫格；`images/` 保存每件物体的 640×480 原图；`render-manifest.json` 记录源 USD、原始单位／坐标轴、尺寸和失败原因。`--group objects` 可只渲染抓取物体，`--limit 3` 可先检查少量模型，`--output-dir` 可指定输出目录。重跑会替换同名预览图。

预览使用 Isaac Sim RTX 和源材质，各格独立居中、缩放取景，**不是相同比例尺**；标注尺寸按源单位与默认姿态下的几何包围盒换算为厘米。物理时间不推进，源文件不修改。图像只用于观察外观，不能替代碰撞、质量和抓放验收。渲染失败会显示失败格并以非零状态退出；排查时查看清单、终端与 Kit 日志。环境文档中已列出的上游启动警告仍可能出现。

## 后续候选

共享目录中还确认了 ManiSkill 厨房台面 GLB、RoboDojo 房间／玩具车／碗，以及 LIBERO 扫描物体与功能区域配置。这些候选尚未完成当前环境的物理验收，因此未注册。RoboTwin 和 RoboCasa 的主要对象包在本地尚未下载。先通过现有少量资产检验接口，再按任务需要逐个加入。

盘子／纸盒的准备、推动、健康检查和重放命令统一见[日常物体推动](implementation.md#日常物体推动盘子与纸盒)。
