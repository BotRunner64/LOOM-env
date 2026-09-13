# USD 空路径警告调查

2026-09-13 对默认 Panda 抓放场景开启 USD 诊断后，原来的“USD Diagnostics are currently muted”展开为一条警告：

```text
Ill-formed SdfPath <>: :1:1: parse error matching ... Sdf_PathParser::Path
```

含义是原生代码将空字符串送入 USD 路径解析器，并非“某个 USD 文件不存在”。本次完整场景初始化中没有发现 USD 资产文件缺失或材质引用无法解析的警告。

## 已定位的触发条件

项目当前 Panda 引用 `Assets/Isaac/5.1/Isaac/IsaacLab/Robots/FrankaEmika/panda_instanceable.usd`，运行环境为 Sim 6.0.1 / Lab 3.0。单独加载这个原始资产即可复现；不需要项目任务、规划器或场景其他物体。

原生栈及 Python 栈确认以下调用链：

```text
SimulationContext.reset()
  → PhysxManager._warmup_and_create_views()
  → force_load_physics_from_usd()
  → PhysX / usdrt.scenegraph
  → usdrt.population / omni.fabric
  → SdfPath(string)
```

警告的直接调用方在 Sim 自带的 Fabric/USDRT 原生插件中。对照实验如下，均保持 USD 诊断开启：

| 场景 | 空路径警告 |
| --- | --- |
| 空物理场景 | 无 |
| 单独加载篮子 | 无 |
| 单独加载原始 Panda | 有 |
| 默认完整 Panda 场景 | 有 |
| 原始 Panda，仅在内存中取消实例化 | 无 |
| 纯代码创建的实例化方块，无外部资产 | 有 |
| 同一方块，仅将 `SetInstanceable(True)` 改为 `False` | 无 |
| Panda 取消实例化后再补齐空材质绑定 | 无；不能据此单独归因于材质绑定 |

进一步用代码创建的实例化方块（无任何外部资产）也复现了相同警告。因此外部 Panda 文件内容、旧版资产或空材质绑定都不是必要触发条件，问题指向当前 Sim 的实例化几何在 Fabric/USDRT 同步中的处理。具体哪个内部实例字段被转换成空路径，尚未定位到原生源码行；没有验证其他 Sim 版本是否已解决。

完整场景仍成功完成初态稳定检查、重置和三相机图像读取；前视图中机器人、桌子、积木、篮子正常可见。该事实不证明全部 Fabric 行为无影响。取消实例化只是诊断实验，可能影响内存占用及运行效率，未作为生产修复实施。

## 最小复现

仓库根目录激活完整 `loom-env` 环境，按[环境说明](environment.md)设置 GPU/Vulkan 并接受 EULA。下面只用代码创建一个方块，不需要下载机器人或场景资产。启动参数只对本次进程打开诊断，并关闭设置持久化，不保存全局用户偏好。

```bash
TF_DEBUG=TF_LOG_STACK_TRACE_ON_WARNING python - <<'PY'
from isaaclab.app import AppLauncher

launcher = AppLauncher(
    headless=True, enable_cameras=True,
    kit_args='--/persistent/app/usd/muteUsdDiagnostics=false --/app/settings/persistent=false',
)
try:
    import isaaclab.sim as sim_utils
    from isaaclab_physx.physics import PhysxCfg
    from pxr import UsdGeom, UsdPhysics
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        device='cuda:0', physics=PhysxCfg(enable_external_forces_every_iteration=True),
    ))
    UsdGeom.Xform.Define(sim.stage, '/World/Template')
    cube = UsdGeom.Cube.Define(sim.stage, '/World/Template/Cube')
    cube.GetSizeAttr().Set(0.1)
    body = UsdGeom.Xform.Define(sim.stage, '/World/Body').GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    visual = UsdGeom.Xform.Define(sim.stage, '/World/Body/Visual').GetPrim()
    visual.GetReferences().AddInternalReference('/World/Template')
    visual.SetInstanceable(True)  # 对照实验只把 True 改为 False。
    print('BEFORE reset', flush=True)
    sim.reset()
    print('AFTER reset', flush=True)
finally:
    launcher.app.close()
PY
```

原始配置预期在两个标记之间出现 `Ill-formed SdfPath <>`，随后 reset 完成。取消实例化的对照中该警告消失。其他已知初始化警告仍会出现，不属于本次对照指标。`TF_DEBUG` 会打印原生栈文件路径，可从该文件读取 Python 栈；无需调试器。

本轮证据位于已忽略的 `.cache/checks/usd-diagnostics/`：

- `scene.log`：完整场景展开后的诊断；`front.png`、`left_wrist.png`、`right_wrist.png`：初态图像。
- `empty.log`、`basket.log`、`panda.log`：空场景与单资产对照。
- `panda-stack.txt`、`resolved-stack.txt`：Panda 警告原生栈及对应插件地址映射。
- `instances.log`、`bindings.log`：仅取消实例化与额外补齐绑定的对照。
- `synthetic.log`、`synthetic-plain.log`：纯代码方块开启／关闭实例化的对照。方块未添加碰撞体和质量配置，可能额外出现质量／惯量提示，不影响单独比较本条 SdfPath 警告。

调查中的全量属性读取另产生了 USD 诊断计数上限提示（`panda-inspect.log`），该轮不能用于判断 reset 后是否有警告；上表结论使用独立、未达到诊断上限的运行。
