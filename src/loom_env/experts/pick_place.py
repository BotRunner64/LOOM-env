"""Complete lift/place demonstrations assembled from feedback actions."""

from scipy.spatial.transform import Rotation

from loom_env.assets.catalog import asset_definition
from loom_env.runtime.runner import SourceFailure
from loom_env.scenes.workspace import transform, workspace

from .actions import ActionExpert, Extract, Grasp, Move, MoveHeld, Release


def lift_from_support(arm, support):
    """Choose a lift or fixture extraction from the scene's initial support."""
    fixture = arm.scene.objects[arm.obj].get("initial_fixture")
    if fixture:
        socket = asset_definition(arm.scene.objects[fixture]["asset"]).insertion_socket
        if socket is None:
            raise ValueError("Fixture extraction requires an annotated exit frame")
        frame = transform(arm.world[f"{fixture}/pose_world"], socket.pose)
        return Extract(
            Rotation.from_quat(frame[3:]).apply([0, 0, 1]), contact_objects=(fixture,)
        )
    if arm.asset.grasp is None:
        raise ValueError("Grasp requires a reviewed grasp annotation")
    point = transform(
        arm.world[f"{arm.obj}/pose_world"], [*arm.asset.grasp, 0, 0, 0, 1]
    )[:3]
    point[2] = support[2] + 0.18 + arm.asset.grasp[2]
    return MoveHeld(
        arm.tcp_at(point), name="lift", allow_object_contact=True, attach=False
    )


def initial_contacts(arm):
    fixture = arm.scene.objects[arm.obj].get("initial_fixture")
    return (fixture,) if fixture else ()


class LiftExpert(ActionExpert):
    def routine(self):
        arm = self.arm
        support, _ = workspace(self.collection.scene)
        yield Grasp(contact_objects=initial_contacts(arm))
        yield lift_from_support(arm, support)
        self.holding = (arm,)


class PickPlaceExpert(ActionExpert):
    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state)
        self.receptacle = collection.role_bindings["container"]
        self.container = asset_definition(
            collection.scene.objects[self.receptacle]["asset"]
        )
        if self.container.interior is None:
            raise ValueError("Expert requires a reviewed container interior")

    def place_goal(self, *, transfer):
        arm = self.arm
        region = arm.world[f"{self.receptacle}/region_pose_world"]
        local = [
            0,
            0,
            self.container.interior[1][2] / 2 - arm.asset.bounds[0][2] + 0.015,
            0,
            0,
            0,
            1,
        ]
        point = transform(region, local)[:3]
        point[2] += arm.asset.grasp[2]
        if transfer:
            held = transform(
                arm.world[f"{arm.obj}/pose_world"], [*arm.asset.grasp, 0, 0, 0, 1]
            )[:3]
            point[2] = max(point[2], held[2])
        return arm.tcp_at(point)

    def routine(self):
        arm = self.arm
        support, _ = workspace(self.collection.scene)
        yield Grasp(contact_objects=initial_contacts(arm))
        yield lift_from_support(arm, support)
        if arm.world[f"{arm.obj}/pose_world"][2] < support[2] + 0.10 or not arm.held:
            raise SourceFailure("Object was not physically lifted", kind="skill")
        yield MoveHeld(self.place_goal(transfer=True), name="transfer")
        retreat = arm.tcp.copy()
        yield MoveHeld(self.place_goal(transfer=False), name="lower")
        yield Release()
        yield Move(retreat, name="retreat")
