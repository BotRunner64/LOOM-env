"""Measured parallel-jaw grasp evidence; no commanded action or task stage."""

import numpy as np


def opposing_contacts(forces, opening, command_limits):
    forces = np.asarray(forces)
    if forces.shape != (2, 3) or not np.isfinite(forces).all():
        raise ValueError("Expected two finite finger contact force vectors")
    magnitudes = np.linalg.norm(forces, axis=1)
    low, high = command_limits
    if not np.isfinite(opening):
        raise ValueError("Expected finite measured gripper opening")
    return bool(
        np.all(magnitudes > 0.2)
        and np.dot(forces[0], forces[1]) < -0.5 * np.prod(magnitudes)
        and 0.05 < (opening - low) / (high - low) < 0.9875
    )
