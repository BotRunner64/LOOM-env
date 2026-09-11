"""Deployment-derived commands and gripper measurements shared across runtimes."""

import numpy as np

from loom_env.specs.config import ARMS


def initial_command(deployment):
    return np.concatenate(
        [
            np.r_[
                deployment.arms[side].initial_positions,
                [hi for _, hi in deployment.arms[side].gripper.command_limits],
            ]
            for side in ARMS
        ]
    )


def gripper_command(gripper, position):
    """Recover a scalar command coordinate from the declared affine joint map."""
    matrix = np.asarray(gripper.joint_map)
    if matrix.shape[1] != 1 or np.linalg.matrix_rank(matrix) != 1:
        raise ValueError("Expected one observable gripper command")
    return float(
        np.linalg.lstsq(
            matrix, position - np.asarray(gripper.joint_offset), rcond=None
        )[0][0]
    )


def gripper_mapping_error(gripper, positions):
    """Measure linkage consistency independently of its common command lag."""
    matrix = np.asarray(gripper.joint_map)
    relative = np.asarray(positions) - gripper.joint_offset
    commands = np.linalg.lstsq(matrix, relative.T, rcond=None)[0].T
    return np.max(np.abs(relative - commands @ matrix.T))
