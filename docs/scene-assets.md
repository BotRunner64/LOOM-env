# 场景资产接入与扩展

首批选用 ManiSkill 木桌、RoboDojo 积木和收纳篮，保留真实模型与材质。资产来源、固定 revision、源文件 SHA-256、单位、尺寸和功能标注集中在 [资产目录](../src/loom_env/assets/catalog.py)，本文不重复维护版本值。

## 已接入资产

| 资产 ID | 用途 | 碰撞与功能 |
| --- | --- | --- |
| `maniskill:table` | 桌面工作区 | 源 GLB 转 USD，桌面顶中心作为原点；桌板为长方体，桌腿保留静态三角网格 |
| `robodojo:brick` | 动态操作对象 | 保留源动态刚体和凸分解碰撞；补充 Panda 抓取中心 |
| `robodojo:basket` | 动态目标容器 | 保留源动态刚体和 SDF 碰撞；记录局部放置区域，随实测容器位姿更新 |

源模型来自本机 `/inspire/hdd/global_user/czxs253130598/projects/sim_projects`。RoboDojo 的 `Assets/` 是下载资产仓库的链接。资产准备入口通过 `--source-root` 指向共享目录，缓存统一写入被 Git 忽略的 `.cache/assets/scenes/`，共享源文件保持原样。

ManiSkill GLB 通过 `loom_env.assets.convert` 调用 Isaac Lab 的标准 MeshConverter，保留外观。该源文件没有物理定义；桌板碰撞使用资产目录中实测包围尺寸的长方体，桌腿及下方横梁保留原三角网格。准备入口必须找到唯一匹配测量范围的桌板组件才会替换。原桌板三角网格与篮子 SDF 的接触导致持续摆动；长方体提供稳定平面。规划碰撞缓存由同一长方体和下方网格生成，与实际桌子碰撞一致。

RoboDojo USDZ 按字节复制后直接引用，保留刚体层级、凸分解／SDF、质量、摩擦等已有物理配置。加载器不覆盖这些参数。资产目录的动态标记用于校验，场景配置不能把动态源模型改成静态。准备时逐项比较源 USDZ 与缓存引用后的物理 schema 和属性。

规划网格与物理资产分开处理：导出碰撞源网格供 cuRobo 使用，不反向修改 PhysX 的源碰撞配置。源网格与 PhysX 烹饪后的凸分解／SDF 不是同一种表示，规划接触裕量和实际执行仍需验证，不能宣称它们完全一致。

## 质量门槛

资产可打开只是准备的起点。新增资产需要通过以下检查：

1. 固定源文件与哈希，检查材质依赖、坐标轴、单位和实际网格尺寸。
2. 明确静态／动态行为、质量和碰撞方式。容器不能使用填平内腔的单一凸包。
3. 仅补当前任务需要的功能标注，并对照网格测量。物体包围盒不能直接充当容器内腔。
4. 执行 `scripts/check_scene_assets.py`，检查真实物理静置和释放入容器，查看输出的相机图像。
5. 执行对应专家任务及物理重放，再把该资产用于采集。保留失败尝试，不能只保存成功样本。

每份缓存保存完整文件哈希和准备版本；缺失、被修改或定义发生变化时拒绝加载。每个 Episode 保存实际资产版本、稳定后的物理初态和展开配置。

## 如何扩展

**新增资产：** 在 `assets/catalog.py` 增加一个固定来源及所需标注，准备并验收缓存。新导入格式确有需要时才扩展 `assets/prepare.py`；不为每个模型创建独立加载器。

**新增场景：** 新建 `configs/scenes/*.yaml`，引用资产 ID，配置工作区和实例局部位姿／采样范围。通过 collection 引用该场景，并选择匹配的部署。场景采样只处理支撑、范围、分离和坐标变换，不依赖任务 ID 或角色名。可达性仍需专家执行验证。

**新增任务：** 增加任务配置与状态判定类，在 `tasks/__init__.py` 显式注册。已有环境输出所有动态物体位姿、速度和两臂接触证据，以及容器区域；任务通过角色绑定读取目标实例。不同动作流程另加专家并在 `runtime/build.py` 注册，Runner 和存储协议无需修改。

当前工作区限定为水平平面，采样只覆盖受支撑且彼此分离的物体。关节物体、堆叠初态、非矩形功能区域需要对应的新物理状态和判据，应在具体任务要求出现时实现。

## 本地复用的 RoboDojo 源模型

从共享 `sim_projects/RoboDojo/Assets/Object/RoboDojo/` 批量复制桌面物体到 `.cache/assets/robodojo/`，保留各模型的 USDZ、元数据、描述及配套文件。源版本为 `RoboDojo-Benchmark/RoboDojo` 的 `a14409d7fae673c00499e01fd88b4457df6351b1`；共享源文件保持原样。

这是覆盖大量类别与款式的物体库，不再逐个手选少量样例。复制范围和筛选条件只在 [`scripts/copy_robodojo_assets.py`](../scripts/copy_robodojo_assets.py) 中维护：

- 包含 `Rigid` 普通物体、`Geometry` 容器和功能物体、`Clutter` 日用品及场景物体，保留符合条件的全部款式。
- 源元数据的最长边不超过 50 cm；已声明的质量须为正且不超过 3 kg，未声明质量的模型保留并在清单中记录为空。
- 缺少 `object.usdz`、尺寸无效、明显过大或质量不合要求的模型列入清单的 `excluded`，附原因。关节物体、衣物、液体和输送带不在当前刚体桌面资产集合内。

在仓库根目录、激活完整仿真环境后执行，无需网络或 GPU：

```bash
conda activate loom-env
python scripts/copy_robodojo_assets.py
```

默认从本机共享目录读取，可用 `--source-root` 替换源路径、`--output-dir` 替换目标缓存、`--workers` 调整复制并发数（默认 4）。需要约 12 GiB 可用空间。重跑会核对已有文件并继续复制，不覆盖内容不同的目标文件；文件先写临时路径，哈希一致后才替换。

`.cache/assets/robodojo/manifest.json` 保存源版本、模型路径、尺寸／质量、逐文件 SHA-256、排除原因、复制失败及依赖检查结果。命令结束打印数量、大小、失败数和清单路径；退出码 0 表示复制和依赖检查全部通过，非零时先看 `failures` 及各模型的 `dependency_check`。可以直接查看本地清单与模型：

```bash
python -m json.tool .cache/assets/robodojo/manifest.json
# 示例：.cache/assets/robodojo/Rigid/toy_car/00001/object.usdz
```

检查包括源／副本哈希一致、USDZ 包校验和、USD 可打开及引用完整性；`OmniPBR.mdl`、`gltf/pbr.mdl` 等标准材质由已安装的 Isaac Sim 提供，不重复复制。未启动 Kit 的 USD 检查器会报告这些运行时模块无法直接解析，脚本核实本机确有对应模块后单独记录。源模型的质量、碰撞配置和功能标注按原样保留；通过文件检查不代表所有模型都能被当前夹爪抓取，新增对象尚未逐一接入任务或进行物理验收。原有积木和篮子的准备产物仍位于 `.cache/assets/scenes/`。

### RoboDojo 示例宫格

在仓库根目录激活完整环境，按[环境说明](environment.md)配置 EULA、GPU 和 Vulkan 后执行：

```bash
python scripts/render_asset_gallery.py --collection robodojo
```

默认从已复制的模型中渲染 30 个代表条目，选择在 [`configs/assets/robodojo_preview.json`](../configs/assets/robodojo_preview.json) 中维护。输出位于 `.cache/previews/robodojo-sample/`：`food.jpg` 为食品与饮料，`everyday.jpg` 为玩具与日用品，`tools_containers.jpg` 为工具与容器。每格标注类别／型号和源尺寸；独立缩放取景，不是相同比例尺。单模型 PNG 在 `images/`，源路径、实测尺寸与失败信息在 `render-manifest.json`。可用 `--group food`、`--limit 3` 只检查少量条目，或用 `--output-dir` 保存另一份预览。渲染保留源材质、不推进物理时间，也不修改源资产。

## Isaac Sim 官方物体缓存

仅保留适合当前桌面抓放任务的 15 个源模型，清单在 [`assets/isaacsim.py`](../src/loom_env/assets/isaacsim.py) 单处维护，下载和宫格预览共用：

- **12 个抓取物体**：糖盒、番茄汤罐、金枪鱼罐、布丁盒、果冻盒、午餐肉罐、泡沫砖、通心粉包装盒，以及蓝／绿／红／黄四个积木。
- **2 个容器**：YCB 碗、小号 KLT 箱。
- **1 个桌面**：Thorlabs 工作台。

其余官方资产及旧宫格已从本地缓存删除，包括工具、装配零件、大纸箱、大周转箱、魔方、基础几何体、杯子、额外桌台与支架。原有 ManiSkill／RoboDojo 任务资产不受影响。保留模型有物理包装层时优先使用该入口；内部网格、材质及纹理作为依赖保留，不计为独立物体。这些源模型尚未注册到项目任务目录，接入时仍按本文质量门槛检查实际抓放效果。

使用与本机 Isaac Sim 6.0.1 对应的 **6.0 资产根目录**，版本关系见 [NVIDIA 资产设置文档](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_faq.html)。在仓库根目录激活完整环境后运行：

```bash
conda activate loom-env
python scripts/download_isaacsim_assets.py
```

默认下载到 `.cache/assets/isaacsim-6.0/`，可用 `--output-dir` 指定缓存、`--workers` 调整下载线程数（默认 8）。下载与依赖检查不启动仿真，不需要 GPU；需要访问 NVIDIA 公开 S3 服务。当前保留模型及依赖共约 307 MiB，建议准备至少 0.5 GB 空间。

- `Isaac/Props/`：保留的源 USD、网格、材质和纹理。
- `download-manifest.json`：源版本地址、入口清单、ETag、大小、更新时间及逐文件 SHA-256。
- `dependency-report.json`：USD 引用、USDZ 包内成员及 MDL 纹理路径的检查结果。

首次运行只获取选定入口，再递归补齐引用。后续运行复用清单并校验文件；重跑会跳过校验通过的文件，未完成的单个文件重新下载。官方版本目录可能更新，复现同一快照需保留并复用下载清单。源文件内容不做修改，Isaac Sim 自带的 `OmniPBR` 由运行环境提供。

退出码 0 表示下载及上述依赖检查通过；1 表示下载失败；2 表示有缺失依赖，查看报告中的 `unresolved`。修改保留清单时应同步更新缓存清单并按依赖闭包清理，不能按文件名删除仍被其他模型引用的纹理。当前保留集合的依赖检查无缺失。

### 渲染物体宫格图

在仓库根目录激活完整 `loom-env` 环境，完成下载，并按[环境文档](environment.md)配置 EULA 和本机 GPU/Vulkan 后运行：

```bash
python scripts/render_asset_gallery.py
```

输出到 `.cache/previews/isaacsim-assets/`：`objects.jpg`、`containers.jpg`、`workspaces.jpg` 为分类宫格；`images/` 保存每件物体的 640×480 原图；`render-manifest.json` 记录源 USD、原始单位／坐标轴、尺寸和失败原因。`--group objects` 可只渲染抓取物体，`--limit 3` 可先检查少量模型，`--output-dir` 可指定输出目录。重跑会替换同名预览图。

预览使用 Isaac Sim RTX 和源材质，各格独立居中、缩放取景，**不是相同比例尺**；标注尺寸按源单位与默认姿态下的几何包围盒换算为厘米。物理时间不推进，源文件不修改。图像只用于观察外观，不能替代碰撞、质量和抓放验收。渲染失败会显示失败格并以非零状态退出；排查时查看清单、终端与 Kit 日志。环境文档中已列出的上游启动警告仍可能出现。

## 后续候选

共享目录中还确认了 ManiSkill 厨房台面 GLB、RoboDojo 房间／玩具车／碗，以及 LIBERO 扫描物体与功能区域配置。这些候选尚未完成当前环境的物理验收，因此未注册。RoboTwin 和 RoboCasa 的主要对象包在本地尚未下载。先通过现有少量资产检验接口，再按任务需要逐个加入。
