# Protobuf health.proto 重复注册调查

2026-09-13 在当前安装的 Sim 6.0.1 / Lab 3.0 环境中确认：两个 NVIDIA 原生插件都编入了 `grpc/health/v1/health.proto` 的生成代码。它们在同一进程中加载时，第二个插件尝试向共享的 Protobuf 注册表注册同名文件，产生 error 和 warning。

涉及文件均在当前 Python 环境的 `lib/python3.12/site-packages/isaacsim/extscache/` 下：

- `omni.grpc.lib-1.0.0+f9bf0dda.lx64.r/bin/libomni.grpc.lib.so`
- `omni.datastore-0.0.0+f9bf0dda.lx64.r/bin/libomni.datastore.ext.plugin.so`

两个库均导出 `descriptor_table_grpc_2fhealth_2fv1_2fhealth_2eproto` 相关符号；都依赖 `libprotobuf.so.26.0.0`。datastore 扩展声明依赖 grpc 扩展，所以正常 Kit 启动会加载两者。它用于派生数据缓存，不能因为采集代码没有主动使用 gRPC 就推断这个扩展不会启动。

安装包归属检查确认：两个文件均来自 `isaacsim-extscache-kit==6.0.1.0`，SHA-256 都与该 wheel 的 RECORD 一致。因此本次不是这两个文件被本地修改造成的差异。

## 因果验证

| 实验 | 结果 |
| --- | --- |
| 仅 `import isaacsim` | 无该错误 |
| `AppLauncher(headless=True, enable_cameras=True)`，不导入项目代码、不创建场景 | 复现该错误 |
| 仅加载 grpc 插件及原生支持库 | 无该错误，退出 0 |
| 仅加载 datastore 插件及原生支持库 | 无该错误，退出 0 |
| 先 grpc，再 datastore | 加载 datastore 时复现，退出 0 |
| 先 datastore，再 grpc | 加载 grpc 时复现，退出 0 |

因此，项目采集逻辑、机器人资产、cuRobo、相机渲染配置以及 Python 的 `google.protobuf` / `grpc` 导入均不是必要触发条件。更换这两个插件的加载顺序不能消除重复注册。

为什么没有中断：安装的 `omni.protobuf.lib/config/extension.toml` 明确说明它使用修改过的 Protobuf v5.26.0，允许重复注册而不失败。最小复现也验证了打印 error/warning 后继续返回、进程退出为 0。这解释了容错行为，不代表已证明所有 Protobuf 消费方都不受影响。

调查产物在已忽略的 `.cache/checks/protobuf/`：`startup.log`、`matching-libraries.txt`、`matrix.json`、四个库加载日志，以及安装包完整性检查 `package-integrity.json`。本轮没有修改依赖、插件或生产启动配置。

## 最小复现

从仓库根目录激活现有完整 `loom-env` 环境后执行。无需机器人资产、GPU 仿真或应用启动；实验在独立子进程中加载当前安装的原生库，依赖搜索路径仅对该子进程生效。

```bash
python - <<'PY'
import os
from pathlib import Path
import subprocess
import sys
import sysconfig

sim = Path(sysconfig.get_paths()['purelib']) / 'isaacsim'
exts = sim / 'extscache'
grpc = next(exts.glob('omni.grpc.lib-*/bin'))
protobuf = next(exts.glob('omni.protobuf.lib-*/bin/protobuf/libprotobuf.so'))
datastore = next(exts.glob('omni.datastore-*/bin/libomni.datastore.ext.plugin.so'))
plugins = {'grpc': grpc / 'libomni.grpc.lib.so', 'datastore': datastore}
env = os.environ.copy()
env['LD_LIBRARY_PATH'] = os.pathsep.join(
    [str(grpc), str(grpc / 'grpc'), env.get('LD_LIBRARY_PATH', '')]
)
for order in [('grpc',), ('datastore',), ('grpc', 'datastore'), ('datastore', 'grpc')]:
    code = 'import ctypes\nhandles = []\n'
    for path in [sim / 'kit/libcarb.so', protobuf]:
        code += f'handles.append(ctypes.CDLL({str(path)!r}, mode=ctypes.RTLD_GLOBAL))\n'
    for name in order:
        code += f'print("BEFORE {name}", flush=True)\n'
        code += f'handles.append(ctypes.CDLL({str(plugins[name])!r}, mode=ctypes.RTLD_LOCAL))\n'
        code += f'print("AFTER {name}", flush=True)\n'
    print('\nCASE', order, flush=True)
    result = subprocess.run([sys.executable, '-u', '-c', code], env=env)
    print('EXIT', result.returncode, flush=True)
PY
```

预期：单独加载无重复注册，组合加载均出现 `File already exists in database` 和 `File is already registered`，四个子进程退出码均为 0。如果报缺失 `.so`，先检查激活的环境、扩展目录及依赖搜索路径；这不算复现成功。

## 处理边界

从根因看，修复应在原生插件的构建／链接层消除重复协议定义，或采用已经解决此问题的上游扩展组合。当前没有验证其他版本是否已修复。仅调整 Python protobuf 包版本、日志等级或插件加载顺序，不能作为本次原因对应的已验证修复。禁用 datastore 会改变渲染缓存依赖，需要另行验证，未在本轮实施。
