"""Explicit assembly shared by collection, replay, and physical validation."""

from loom_env.experts.handover import HandoverExpert
from loom_env.experts.pick_place import LiftExpert, PickPlaceExpert
from loom_env.experts.push import PushExpert
from loom_env.experts.sweep import SweepExpert
from loom_env.tasks import create_task

EXPERTS = {
    "put_object_in_container": PickPlaceExpert,
    "lift_object": LiftExpert,
    "push_object": PushExpert,
    "push_into_region": PushExpert,
    "handover_object": HandoverExpert,
    "sweep_into_region": SweepExpert,
}


def create_environment(collection, asset_root):
    from loom_env.environments.isaac_lab import ManipulationEnvironment

    create_task(collection)  # Validate the complete combination before physics.
    return ManipulationEnvironment(collection, asset_root)


def create_expert(collection, environment, asset_root):
    from loom_env.experts.curobo import ArmPlanner

    create_task(collection)
    try:
        factory = EXPERTS[collection.task.id]
    except KeyError as error:
        raise ValueError(f"No expert for task: {collection.task.id}") from error
    planners = {}
    try:
        sides = (
            (collection.arm_roles["giver"], collection.arm_roles["receiver"])
            if factory is HandoverExpert
            else (collection.arm_roles["manipulator"],)
        )
        for side in sides:
            planners[side] = ArmPlanner(
                collection.deployment,
                collection.scene,
                side,
                collection.role_bindings[
                    "tool" if factory is SweepExpert else "target_object"
                ],
                asset_root,
            )
        planning = planners if factory is HandoverExpert else planners[sides[0]]
        return factory(collection, planning, environment.world_state)
    except BaseException:
        for planner in planners.values():
            planner.planner.destroy()
        raise
