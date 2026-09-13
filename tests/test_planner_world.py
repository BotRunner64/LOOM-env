"""Exercise incremental scene updates with cuRobo's real GPU collision queries."""

from types import SimpleNamespace

import pytest


def test_world_updates_move_obstacles_and_restore_target_collision():
    torch = pytest.importorskip("torch")
    pytest.importorskip("curobo")
    if not torch.cuda.is_available():
        pytest.skip("cuRobo scene queries require CUDA")
    trimesh = pytest.importorskip("trimesh")
    import numpy as np
    from scipy.spatial.transform import Rotation
    from curobo.scene import Mesh
    from curobo._src.geom.collision.buffer_collision import CollisionBuffer
    from curobo._src.geom.collision.collision_scene import (
        SceneCollision,
        SceneCollisionCfg,
    )
    from curobo._src.types.device_cfg import DeviceCfg
    from loom_env.experts.curobo import ArmPlanner

    device = DeviceCfg()
    checker = SceneCollision.from_config(
        SceneCollisionCfg(device_cfg=device, cache={"obb": 4, "mesh": 4})
    )
    loads, resets = [], []

    def load(scene):
        loads.append(scene)
        checker.load_collision_model(scene)
        resets.append(True)

    planner = ArmPlanner.__new__(ArmPlanner)
    planner.side = "right"
    planner.target_object = "target"
    planner.base = np.array([0, 0, 0, 0, 0, 0, 1])
    planner.rotation = Rotation.identity()
    planner.scene = SimpleNamespace(objects={"table": {}, "target": {}})
    planner.deployment = SimpleNamespace(
        arms={"left": SimpleNamespace(base_pose=planner.base)}
    )
    geometry = trimesh.creation.box(extents=[0.1, 0.1, 0.1])
    planner.meshes = {
        name: Mesh(
            name=name,
            vertices=geometry.vertices.tolist(),
            faces=geometry.faces.tolist(),
            pose=[0, 0, 0, 1, 0, 0, 0],
        )
        for name in planner.scene.objects
    }
    planner.planner = SimpleNamespace(
        scene_collision_checker=checker,
        update_world=load,
        graph_planner=SimpleNamespace(reset_buffer=lambda: resets.append(True)),
    )
    planner.holding_model = SimpleNamespace(
        compute_kinematics=lambda q: SimpleNamespace(
            robot_spheres=torch.tensor([[q[0], 0, 0, 0.05]], device="cuda")
        )
    )
    planner._state = lambda q, arm: q
    planner.attached = planner.world_loaded = False
    observation = SimpleNamespace(values={"robot/left/joint_position": [-1.0]})
    truth = {
        "table/pose_world": np.array([0, 0, 0, 0, 0, 0, 1.0]),
        "target/pose_world": np.array([1, 0, 0, 0, 0, 0, 1.0]),
    }
    query = torch.tensor(
        [[[[x + 0.055, 0, 0, 0.03] for x in [-2, -1, 0, 1, 2]]]],
        device="cuda",
    )
    buffer = CollisionBuffer.from_shape(query.shape, device)

    def check(allow, expected, count):
        assert (
            planner.update_world(observation, truth, allow_object_contact=allow)
            == count
        )
        cost = checker.get_sphere_distance(
            SimpleNamespace(robot_spheres=query),
            buffer,
            torch.tensor([1.0], device="cuda"),
            torch.tensor([0.005], device="cuda"),
        )
        assert (cost.detach().flatten() > 1e-6).tolist() == expected

    # The first call may already permit contact; the target must still be loaded.
    check(True, [False, True, True, False, False], 2)
    check(False, [False, True, True, True, False], 3)
    observation.values["robot/left/joint_position"] = [-2.0]
    truth["target/pose_world"][0] = 2.0
    check(False, [True, False, True, False, True], 3)
    planner.attached = True
    check(False, [True, False, True, False, False], 2)
    planner.attached = False
    check(False, [True, False, True, False, True], 3)
    assert len(loads) == 1
    assert len(resets) == 5
