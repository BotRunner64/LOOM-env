"""Deployment-derived control commands shared by tasks and diagnostics."""

import numpy as np


def initial_command(deployment):
    return np.concatenate(
        [
            np.r_[
                deployment.arms[side].initial_positions,
                [hi for _, hi in deployment.arms[side].gripper.command_limits],
            ]
            for side in ("left", "right")
        ]
    )
