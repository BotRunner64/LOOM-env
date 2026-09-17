"""Measured parallel-jaw grasp evidence; no commanded action or task stage."""

import numpy as np


def opposing_contacts(forces):
    forces = np.asarray(forces)
    if forces.shape != (2, 3) or not np.isfinite(forces).all():
        raise ValueError("Expected two finite finger contact force vectors")
    magnitudes = np.linalg.norm(forces, axis=1)
    return bool(
        np.all(magnitudes > 0.2)
        and np.dot(forces[0], forces[1]) < -0.5 * np.prod(magnitudes)
    )
