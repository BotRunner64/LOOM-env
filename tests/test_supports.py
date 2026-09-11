from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from loom_env.embodiments import supports
from loom_env.specs.config import load_deployment

ROOT = Path(__file__).parents[1]


@pytest.fixture
def support_source(tmp_path, monkeypatch):
    source = tmp_path / "assembly.urdf"
    source.write_text("""<robot name="assembly">
<link name="openarm_body_link0"/><link name="openarm_left_link0"/><link name="openarm_right_link0"/>
<joint name="left_mount" type="fixed"><parent link="openarm_body_link0"/><child link="openarm_left_link0"/><origin xyz="0 .031 .698" rpy="-1.5708 0 0"/></joint>
<joint name="right_mount" type="fixed"><parent link="openarm_body_link0"/><child link="openarm_right_link0"/><origin xyz="0 -.031 .698" rpy="1.5708 0 0"/></joint>
</robot>""")
    monkeypatch.setattr(supports, "source_urdf", lambda *_: source)
    monkeypatch.setattr(supports, "verify_asset", lambda *_: {})
    return tmp_path


def test_official_shoulders_resolve_to_one_pedestal(support_source):
    deployment = load_deployment(ROOT / "configs/deployments/dual_openarm.yaml")
    support = supports.fixed_support(deployment, support_source)
    np.testing.assert_allclose(support.pose, [-0.15, 0, 0.18, 0, 0, 0, 1], atol=1e-12)


def test_detached_shoulder_is_rejected(support_source):
    deployment = load_deployment(ROOT / "configs/deployments/dual_openarm.yaml")
    right = deployment.arms["right"]
    shifted = list(right.base_pose)
    shifted[1] += 0.1
    deployment = replace(
        deployment,
        arms={**deployment.arms, "right": replace(right, base_pose=tuple(shifted))},
    )
    with pytest.raises(ValueError, match="shared pedestal"):
        supports.fixed_support(deployment, support_source)


def test_other_embodiments_need_no_support_asset(tmp_path):
    deployment = load_deployment(ROOT / "configs/deployments/dual_piper.yaml")
    assert supports.fixed_support(deployment, tmp_path) is None


def test_riser_fills_floor_to_raised_pedestal(support_source):
    mesh = support_source / "openarm_support/meshes/collision.npz"
    mesh.parent.mkdir(parents=True)
    np.savez(mesh, vertices=[[-0.155, -0.095, 0], [0.095, 0.095, 0.773]])
    deployment = load_deployment(ROOT / "configs/deployments/dual_openarm.yaml")
    support = supports.fixed_support(deployment, support_source)
    size, pose = support.riser()
    np.testing.assert_allclose(size, [0.25, 0.19, 0.18], atol=1e-12)
    np.testing.assert_allclose(pose, [-0.18, 0, 0.09, 0, 0, 0, 1], atol=1e-12)
    assert pose[2] - size[2] / 2 == pytest.approx(0)
    assert pose[2] + size[2] / 2 == pytest.approx(support.pose[2])
