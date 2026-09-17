"""Read fixed single-joint object geometry and physics from the authored USD."""

import numpy as np
from scipy.spatial.transform import Rotation

from .prepare import mesh_arrays, physics_properties


def pose_from_matrix(matrix):
    matrix = np.asarray(matrix)
    return np.r_[matrix[3, :3], Rotation.from_matrix(matrix[:3, :3].T).as_quat()]


def describe(stage, definition, directory=None, root=None):
    from pxr import Usd, UsdGeom, UsdPhysics

    root = stage.GetDefaultPrim() if root is None else root
    root_inverse = np.linalg.inv(
        np.array(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0))
    )
    bodies = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    expected = {definition.root_body, definition.moving_body}
    if {p.GetName() for p in bodies} != expected or len(bodies) != 2:
        raise ValueError(
            "Reviewed articulation asset must have exactly the two named bodies"
        )
    result = {"bodies": {}, "root_body": definition.root_body}
    for body in bodies:
        properties = physics_properties(stage, root=body)
        if not properties["materials"]:
            raise ValueError(f"Body has no collision: {body.GetPath()}")
        matrix = np.array(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0))
        result["bodies"][body.GetName()] = {
            "pose_asset": pose_from_matrix(matrix @ root_inverse).tolist(),
            "physics": properties,
        }
        if directory is not None:
            vertices, faces = mesh_arrays(
                [p for p in Usd.PrimRange(body) if p.HasAPI(UsdPhysics.CollisionAPI)]
            )
            vertices = (
                np.c_[vertices, np.ones(len(vertices))] @ np.linalg.inv(matrix)
            )[:, :3]
            np.savez_compressed(
                directory / f"collision-{body.GetName()}.npz",
                vertices=vertices,
                faces=faces,
            )
    joints = [
        p
        for p in Usd.PrimRange(root)
        if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)
    ]
    if len(joints) != 1 or joints[0].GetName() != definition.joint:
        raise ValueError("Expected the reviewed single revolute or prismatic joint")
    revolute = joints[0].IsA(UsdPhysics.RevoluteJoint)
    joint = (UsdPhysics.RevoluteJoint if revolute else UsdPhysics.PrismaticJoint)(
        joints[0]
    )
    for rel, name in [
        (joint.GetBody0Rel(), definition.root_body),
        (joint.GetBody1Rel(), definition.moving_body),
    ]:
        if rel.GetTargets() != [root.GetPath().AppendChild(name)]:
            raise ValueError("Joint connects unexpected bodies")
    fixed = [
        UsdPhysics.FixedJoint(p)
        for p in Usd.PrimRange(root)
        if p.IsA(UsdPhysics.FixedJoint)
    ]
    if (
        len(fixed) != 1
        or fixed[0].GetBody0Rel().GetTargets()
        or fixed[0].GetBody1Rel().GetTargets()
        != [root.GetPath().AppendChild(definition.root_body)]
    ):
        raise ValueError("Expected one world-fixed root body")
    rotation = joint.GetLocalRot0Attr().Get()
    result["joint"] = {
        "name": definition.joint,
        "parent": definition.root_body,
        "child": definition.moving_body,
        "pose_parent": [
            *joint.GetLocalPos0Attr().Get(),
            *rotation.GetImaginary(),
            rotation.GetReal(),
        ],
        "axis": {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[
            joint.GetAxisAttr().Get()
        ],
        "type": "revolute" if revolute else "prismatic",
        "unit": "rad" if revolute else "m",
        "limits": (
            np.asarray(
                [joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()]
            )
            * (np.pi / 180 if revolute else 1)
        ).tolist(),
    }
    return result
