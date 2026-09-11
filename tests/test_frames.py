from dataclasses import replace
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from loom_env.embodiments.assets import PANDA_ASSET, prepared_urdf
from loom_env.embodiments.frames import resolve_camera_mount, resolve_fixed_frame
from loom_env.specs.config import load_deployment


@pytest.fixture
def urdf(tmp_path):
    path = prepared_urdf(tmp_path, "piper")
    path.parent.mkdir(parents=True)
    path.write_text(f'''<robot name="fixture">
      <link name="base"/><link name="wrist"/>
      <link name="mount"/><link name="camera"/>
      <joint name="arm" type="revolute">
        <parent link="base"/><child link="wrist"/>
      </joint>
      <joint name="bracket" type="fixed">
        <parent link="wrist"/><child link="mount"/>
        <origin xyz="1 0 0" rpy="0 0 {math.pi / 2}"/>
      </joint>
      <joint name="sensor" type="fixed">
        <parent link="mount"/><child link="camera"/>
        <origin xyz="1 0 0" rpy="{math.pi / 2} 0 0"/>
      </joint>
    </robot>''')
    return path


def test_fixed_chain_rotates_translation_and_optical_offset(urdf, tmp_path):
    dep = load_deployment("configs/deployments/dual_piper.yaml")
    camera = replace(dep.cameras[1], pose=(0, 1, 0, 0.5, -0.5, -0.5, 0.5))
    body, pose = resolve_camera_mount(camera, dep.arms["left"], ["wrist"], tmp_path)
    assert body == "wrist"
    np.testing.assert_allclose(pose[:3], [1, 1, 1], atol=1e-12)
    optical = Rotation.from_quat(pose[3:])
    np.testing.assert_allclose(optical.apply([0, 0, -1]), [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(optical.apply([0, 1, 0]), [1, 0, 0], atol=1e-12)


def test_nearest_retained_frame_prevents_double_transform(urdf):
    body, pose = resolve_fixed_frame(urdf, "camera", ["wrist", "mount"])
    assert body == "mount"
    np.testing.assert_allclose(pose[:3], [1, 0, 0])
    np.testing.assert_allclose(
        Rotation.from_quat(pose[3:]).apply([0, 1, 0]), [0, 0, 1], atol=1e-12
    )


@pytest.mark.parametrize("asset", [PANDA_ASSET, "robotwin:piper"])
def test_explicit_body_mount_needs_no_urdf(collection, tmp_path, asset):
    arm = replace(collection.deployment.arms["left"], asset=asset)
    camera = replace(collection.deployment.cameras[0], parent_frame="left/wrist")
    body, pose = resolve_camera_mount(camera, arm, ["wrist"], tmp_path)
    assert body == "wrist"
    np.testing.assert_array_equal(pose, camera.pose)


@pytest.mark.parametrize(
    "frame,bodies,error",
    [
        ("missing", ["wrist"], "not found"),
        ("camera", ["base"], "moving joint"),
        ("base", [], "no retained ancestor"),
    ],
)
def test_invalid_mount_fails_instead_of_freezing_or_falling_back(
    urdf, frame, bodies, error
):
    with pytest.raises(ValueError, match=error):
        resolve_fixed_frame(urdf, frame, bodies)


def test_mount_cycle_is_rejected(urdf):
    urdf.write_text(
        urdf.read_text().replace('<parent link="wrist"/>', '<parent link="camera"/>')
    )
    with pytest.raises(ValueError, match="Cycle"):
        resolve_fixed_frame(urdf, "camera", ["wrist"])
