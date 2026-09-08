#!/usr/bin/env python3
"""Exercise Isaac Lab's GPU physics and RGB rendering without external assets."""

import os
from pathlib import Path

if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {"Y", "YES", "1"}:
    raise SystemExit(
        "Set OMNI_KIT_ACCEPT_EULA=YES after accepting NVIDIA's Omniverse EULA."
    )

# The application must start before importing simulator-dependent modules.
from isaaclab.app import AppLauncher

launcher = AppLauncher(headless=True, enable_cameras=True)
app = launcher.app
try:
    import torch
    from PIL import Image

    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.envs import ManagerBasedEnv
    from isaaclab.sensors import Camera, CameraCfg
    from isaaclab_physx.physics import PhysxCfg
    from isaaclab_physx.renderers import IsaacRtxRendererCfg

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 120.0, device="cuda:0", physics=PhysxCfg())
    )
    floor = sim_utils.CuboidCfg(
        size=(2.0, 2.0, 0.1),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
    )
    floor.func("/World/Floor", floor, translation=(0.0, 0.0, -0.05))
    light = sim_utils.DomeLightCfg(intensity=2000.0)
    light.func("/World/Light", light)
    cube = RigidObject(
        RigidObjectCfg(
            prim_path="/World/Cube",
            spawn=sim_utils.CuboidCfg(
                size=(0.1, 0.1, 0.1),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.8, 0.1, 0.1)
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.5)),
        )
    )
    camera = Camera(
        CameraCfg(
            prim_path="/World/Camera",
            height=64,
            width=64,
            data_types=["rgb"],
            renderer_cfg=IsaacRtxRendererCfg(),
            spawn=sim_utils.PinholeCameraCfg(clipping_range=(0.01, 10.0)),
        )
    )
    sim.reset()
    cube.reset()
    camera.reset()
    camera.set_world_poses_from_view(
        eyes=torch.tensor([[1.0, 1.0, 1.0]], device="cuda"),
        targets=torch.tensor([[0.0, 0.0, 0.1]], device="cuda"),
    )
    for _ in range(120):
        sim.step(render=True)
        cube.update(sim.get_physics_dt())
        camera.update(sim.get_physics_dt())
    height = cube.data.root_pos_w.torch[0, 2].item()
    rgb = camera.data.output["rgb"].torch[0, ..., :3]
    assert 0.03 < height < 0.08, f"Unexpected settled cube height: {height}"
    assert rgb.shape == (64, 64, 3), rgb.shape
    assert torch.isfinite(rgb.float()).all() and rgb.float().std() > 0
    output_dir = Path(
        os.environ.get(
            "LOOM_CHECK_OUTPUT_DIR",
            Path(__file__).resolve().parents[1] / ".cache" / "checks",
        )
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgb.cpu().numpy()).save(output_dir / "simulation-rgb.png")
    print(
        f"ISAACLAB_PHYSX_RGB_PASS height={height:.4f} rgb={tuple(rgb.shape)} "
        f"env_class={ManagerBasedEnv.__name__}",
        flush=True,
    )
finally:
    app.close()
