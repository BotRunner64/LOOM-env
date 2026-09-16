# LOOM-env

面向 context-conditioned VLA 的机器人操作仿真与数据框架，用于研究模型如何利用示范和历史经验。基于 Isaac Lab / Isaac Sim / PhysX，专家使用 cuRobo 规划。

## 当前目标

**独立配置任务、场景和部署，组合成可运行、可复现的案例。**

```mermaid
flowchart LR
    T[任务：做什么] --> C[组合与角色绑定]
    S[场景：对象和布局] --> C
    D[部署：机器人、安装、控制、相机] --> C
    C --> R[运行与判定]
    R --> O[轨迹、视频、结果]
```

## 当前进度

- **已具备：** 三类配置与角色绑定、统一执行循环、抓起、放入、直推和双臂交接任务、三路相机记录、采集自动生成视频与重放入口。
- **本体支持：** 六类双臂部署及对应布局；见[本体说明](docs/embodiments.md#抓放录制)。
- **组合验收：** 旧版物理资产下五项代表案例已验证任务／场景／部署替换及左右臂；当前资产已更新，任务效果待复验；见[运行指南](docs/implementation.md#单项替换的组合验收)。尚不覆盖全部本体与任务组合，不同本体可能需要调整布局。
- **双臂交接：** 双 Panda 日常物体交接仍在验证中；已修复滑落被误判为成功的问题，尚未通过独立抓稳验收；见[运行指南](docs/implementation.md#日常物体双臂交接)。
- **新增任务：** 盘子推上薄餐垫、纸盒推入可见收纳区，两项固定场景已跑通；完整区域判据与按物体几何调整接触高度，见[运行指南](docs/implementation.md#日常物体推动盘子与纸盒)。积木变体保留为回归。

## 开发入口

- [架构](docs/architecture.md)：职责、执行流程、代码位置。
- [运行](docs/implementation.md)：配置、采集、重放、数据检查；首次使用先[安装环境](docs/environment.md)。
- 扩展：[机器人](docs/embodiments.md) · [场景资产](docs/scene-assets.md)。
- 排查：[运行排查](docs/implementation.md#如何判断与排查)；协作约定：[AGENTS.md](AGENTS.md)。
