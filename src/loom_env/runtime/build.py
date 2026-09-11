"""Explicit assembly shared by collection, replay, and physical validation."""

from loom_env.experts.pick_place import LiftExpert, PickPlaceExpert
from loom_env.tasks import create_task

EXPERTS = {"put_object_in_container": PickPlaceExpert, "lift_object": LiftExpert}


def create_environment(collection, asset_root):
    from loom_env.environments.isaac_lab import ManipulationEnvironment

    create_task(collection)  # Validate the complete combination before physics.
    return ManipulationEnvironment(collection, asset_root)


def create_expert(collection, environment, asset_root):
    from loom_env.experts.curobo import PandaPlanner

    create_task(collection)
    try:
        factory = EXPERTS[collection.task.id]
    except KeyError as error:
        raise ValueError(f"No expert for task: {collection.task.id}") from error
    planner = PandaPlanner(
        collection.deployment,
        collection.scene,
        collection.arm_roles["manipulator"],
        collection.role_bindings["target_object"],
        asset_root,
    )
    try:
        return factory(collection, planner, environment.world_state)
    except BaseException:
        planner.planner.destroy()
        raise
