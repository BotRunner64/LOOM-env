"""Measured parallel-jaw grasp evidence; no commanded action or task stage."""

import numpy as np


def opposing_contacts(forces, finger_positions, max_width=0.08):
    forces = np.asarray(forces)
    if forces.shape != (2, 3) or not np.isfinite(forces).all():
        raise ValueError("Expected two finite finger contact force vectors")
    magnitudes = np.linalg.norm(forces, axis=1)
    width = float(np.sum(finger_positions))
    return bool(
        np.all(magnitudes > 0.2)
        and np.dot(forces[0], forces[1]) < -0.5 * np.prod(magnitudes)
        and 0.004 < width < max_width - 0.001
    )
