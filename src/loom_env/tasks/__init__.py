"""Explicit task factories; no simulator or expert dependency."""

from .lift import LiftTask
from .place import PlaceTask
from .push import PushTask

TASKS = {
    "put_object_in_container": PlaceTask,
    "lift_object": LiftTask,
    "push_object": PushTask,
}


def create_task(collection):
    try:
        factory = TASKS[collection.task.id]
    except KeyError as error:
        raise ValueError(f"Unsupported task: {collection.task.id}") from error
    return factory(collection)
