"""Explicit task factories; no simulator or expert dependency."""

from .articulation import LaptopHingeTask
from .handover import HandoverTask
from .insertion import CoinInsertionTask
from .lift import LiftTask
from .place import PlaceTask
from .push import PushTask
from .push_region import PushIntoRegionTask
from .sweep import SweepTask

TASKS = {
    "open_laptop": LaptopHingeTask,
    "close_laptop_partway": LaptopHingeTask,
    "insert_coin": CoinInsertionTask,
    "put_object_in_container": PlaceTask,
    "lift_object": LiftTask,
    "push_object": PushTask,
    "push_into_region": PushIntoRegionTask,
    "handover_object": HandoverTask,
    "sweep_into_region": SweepTask,
}


def create_task(collection):
    try:
        factory = TASKS[collection.task.id]
    except KeyError as error:
        raise ValueError(f"Unsupported task: {collection.task.id}") from error
    return factory(collection)
