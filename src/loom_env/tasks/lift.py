"""Lift the entire target clear of its support while physically holding it."""

from .object_task import ObjectTask


class LiftTask(ObjectTask):
    def __init__(self, collection):
        super().__init__(collection, ("clearance",))
        if set(collection.role_bindings) != {"target_object"}:
            raise ValueError("Lift requires only target_object")

    def update(self, world_state, dt):
        points, grasped = self._state(world_state)
        lifted = points[:, 2].min() >= self.support[2] + self.parameters["clearance"]
        return self.finish(
            points,
            lifted and grasped.any(),
            dt,
            "object_lifted_and_held",
        )
