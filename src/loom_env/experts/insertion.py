"""Pick a supported coin, align using measured grasp geometry, and insert it."""

import numpy as np
from scipy.spatial.transform import Rotation

from loom_env.runtime.runner import SourceFailure
from loom_env.specs.episode import Action, Event
from loom_env.tasks import create_task

from .pick_place import LiftExpert


class CoinLiftExpert(LiftExpert):
    """Top-down grasp across the exposed faces of the reviewed upright coin."""

    def reset(self, episode_input):
        super().reset(episode_input)
        self.lift_goal = None
        coin = self.world_state()[f"{self.obj}/pose_world"]
        closing = Rotation.from_quat(coin[3:].copy()).apply([0.0, 0.0, 1.0])
        closing[2] = 0
        if np.linalg.norm(closing) < 0.9:
            raise ValueError("Coin grasp requires an upright initial coin")
        closing /= np.linalg.norm(closing)
        down = np.array([0.0, 0.0, -1.0])
        self.tool_rotation = Rotation.from_matrix(
            np.column_stack((np.cross(closing, down), closing, down))
        )

    def _lift_step(self, observation):
        world, events = self.world_state(), []
        held = bool(world[f"{self.obj}/grasped_by"][0 if self.side == "left" else 1])
        self.lost_contact = 0 if held else self.lost_contact + 1
        if self.lost_contact >= 5:
            raise SourceFailure("Coin slipped during slow extraction", kind="skill")
        tcp = observation.values[f"robot/{self.side}/tcp_pose_world"]
        if self.lift_goal is None:
            self.lift_goal = tcp.copy()
            self.lift_reference_z = float(tcp[2])
            self.lift_goal[2] += 0.125
            self.started = True
            events.append(
                Event("skill", "slow_extraction", self.step, {"speed_m_s": 0.01})
            )
        remaining = self.lift_goal[2] - tcp[2]
        if remaining < 0.001:
            self._advance(events)
        else:
            goal = self.lift_goal.copy()
            self.lift_reference_z = min(
                self.lift_goal[2],
                self.lift_reference_z + 0.01 * self.dt,
                float(tcp[2]) + 0.003,
            )
            goal[2] = self.lift_reference_z
            fixture = self.collection.scene.objects[self.obj].get("initial_fixture")
            self.command[self.arm_slice] = self.planner.cartesian_step(
                observation,
                world,
                goal,
                contact_objects=(fixture,) if fixture else (),
            )
        if self.stage_steps * self.dt > 20:
            raise SourceFailure("Slow coin extraction timed out", kind="skill")
        self.stage_steps += 1
        self.step += 1
        return Action(self.command.copy(), tuple(events))

    def act(self, observation):
        if self.stage == "lift":
            return self._lift_step(observation)
        return super().act(observation)


class CoinInsertionExpert(CoinLiftExpert):
    STAGES = (
        "approach",
        "descend",
        "close",
        "lift",
        "transfer",
        "align",
        "insert",
        "release",
        "retreat",
        "wait",
    )

    def __init__(self, collection, planner, world_state):
        super().__init__(collection, planner, world_state)
        self.task = create_task(collection)

    def reset(self, episode_input):
        super().reset(episode_input)
        self.desired_rotation = None
        self.best_depth = -np.inf
        self.stalled = 0

    def _insertion_goal(self, observation, world, depth):
        coin = world[f"{self.obj}/pose_world"]
        coin_r = Rotation.from_quat(coin[3:].copy())
        tcp = observation.values[f"robot/{self.side}/tcp_pose_world"]
        tcp_r = Rotation.from_quat(tcp[3:].copy())
        fixture = world[f"{self.task.target}/pose_world"]
        fixture_r = Rotation.from_quat(fixture[3:].copy())
        if self.desired_rotation is None:
            normal = coin_r.apply([0, 0, 1])
            desired_normal = fixture_r.apply([0, 1, 0])
            if normal @ desired_normal < 0:
                desired_normal *= -1
            correction, _ = Rotation.align_vectors([desired_normal], [normal])
            self.desired_rotation = correction * coin_r
        center = (
            fixture_r.apply(
                [*self.task.slot_center, self.task.slot_top + self.task.radius - depth]
            )
            + fixture[:3]
        )
        desired_coin_position = center - self.desired_rotation.apply(self.task.center)
        # Re-read the actual grasp transform; never set the physical coin pose.
        coin_in_tcp = tcp_r.inv().apply(coin[:3] - tcp[:3])
        relative_r = tcp_r.inv() * coin_r
        goal_r = self.desired_rotation * relative_r.inv()
        return np.r_[
            desired_coin_position - goal_r.apply(coin_in_tcp), goal_r.as_quat()
        ]

    def _motion_goal(self, observation, world):
        if self.stage in {"approach", "descend", "lift"}:
            return super()._goal(world, observation)
        if self.stage == "transfer":
            return self._insertion_goal(observation, world, -0.035)
        goal = observation.values[f"robot/{self.side}/tcp_pose_world"].copy()
        goal[2] += 0.08
        return goal

    def act(self, observation):
        if self.stage == "lift":
            return self._lift_step(observation)
        world, events = self.world_state(), []
        m = self.task.metrics(world)
        other = "left" if self.side == "right" else "right"
        other_slice = self.collection.deployment.action_slices[f"{other}/arm"]
        if (
            np.max(
                np.abs(
                    observation.values[f"robot/{other}/joint_position"]
                    - self.command[other_slice]
                )
            )
            > 0.03
        ):
            raise SourceFailure("Holding arm moved outside tolerance", kind="skill")
        if self.stage in {"lift", "transfer", "align", "insert"}:
            self.lost_contact = 0 if m["grasped"] else self.lost_contact + 1
            if self.lost_contact >= 5:
                raise SourceFailure("Coin lost opposing finger contacts", kind="skill")
        if not self.started:
            self.started = True
            print(f"EXPERT step={self.step} stage={self.stage}", flush=True)
            events.append(
                Event(
                    "skill", self.stage, self.step, {"phase": "begin", "arm": self.side}
                )
            )
            if self.stage == "transfer":
                if (
                    not m["grasped"]
                    or m["source_clearance"] < self.task.parameters["lift_clearance"]
                ):
                    raise SourceFailure(
                        "Coin was not physically extracted", kind="skill"
                    )
                self.planner.attach(observation, world, max_cell_size=0.01)
            if self.stage == "release":
                self.planner.detach()
            if self.stage in {"approach", "descend", "lift", "transfer", "retreat"}:
                contact = (
                    (self.task.source,)
                    if self.stage in {"descend", "lift"}
                    else (self.task.target,)
                    if self.stage == "retreat"
                    else ()
                )
                self.path, detail = self.planner.plan(
                    observation,
                    world,
                    self._motion_goal(observation, world),
                    allow_object_contact=self.stage in {"descend", "lift", "retreat"},
                    contact_objects=contact,
                )
                if self.stage == "transfer":
                    # Execute the collision-checked path with fourfold time
                    # scaling to avoid accelerating the thin coin out of the jaws.
                    knots = np.vstack(
                        [
                            observation.values[f"robot/{self.side}/joint_position"],
                            self.path,
                        ]
                    )
                    time = np.arange(1, (len(knots) - 1) * 4 + 1) / 4
                    self.path = np.column_stack(
                        [
                            np.interp(time, np.arange(len(knots)), knots[:, j])
                            for j in range(knots.shape[1])
                        ]
                    )
                    detail["execution_time_scale"] = 4
                events.append(Event("planning", self.stage, self.step, detail))
        if self.stage in {"align", "insert"}:
            align = self.stage == "align"
            depth = -0.004 if align else self.task.parameters["depth"] + 0.002
            ready = self.task.aligned(m) and (
                abs(m["depth"] - depth) < 0.0006
                if align
                else m["depth"] >= self.task.parameters["depth"]
            )
            self.stable = self.stable + 1 if ready else 0
            if self.stable >= 3:
                events.append(Event("planning", "coin_alignment", self.step, m))
                self._advance(events)
            else:
                goal = self._insertion_goal(observation, world, depth)
                tcp = observation.values[f"robot/{self.side}/tcp_pose_world"]
                delta = goal[:3] - tcp[:3]
                # 10 mm/s insertion, 30 mm/s pre-insertion alignment.
                limit = (0.03 if align else 0.01) * self.dt
                goal[:3] = tcp[:3] + delta * min(
                    1, limit / max(np.linalg.norm(delta), 1e-12)
                )
                delta_r = (
                    Rotation.from_quat(goal[3:])
                    * Rotation.from_quat(tcp[3:].copy()).inv()
                ).as_rotvec()
                delta_r *= min(1, 0.025 / max(np.linalg.norm(delta_r), 1e-12))
                goal[3:] = (
                    Rotation.from_rotvec(delta_r) * Rotation.from_quat(tcp[3:].copy())
                ).as_quat()
                self.command[self.arm_slice] = self.planner.cartesian_step(
                    observation,
                    world,
                    goal,
                    contact_objects=(self.task.target,),
                )
                if not align:
                    if m["depth"] > self.best_depth + 0.0002:
                        self.best_depth, self.stalled = m["depth"], 0
                    else:
                        self.stalled += 1
                    if self.stalled * self.dt > 2:
                        raise SourceFailure(
                            f"Coin insertion stalled at {m['depth']:.6f} m",
                            kind="skill",
                        )
        elif self.stage in {"close", "release"}:
            closing = self.stage == "close"
            self.command[self.grip_slice] = self.closed if closing else self.opened
            ready = m["grasped"] if closing else not m["any_grasped"]
            self.stable = self.stable + 1 if ready else 0
            if self.stable * self.dt >= (0.6 if closing else 0.1):
                self._advance(events)
            elif self.stage_steps * self.dt > 3:
                raise SourceFailure(
                    f"Coin {self.stage} feedback timed out", kind="skill"
                )
        elif self.stage != "wait":
            if self.path_index < len(self.path):
                self.command[self.arm_slice] = self.path[self.path_index]
                self.path_index += 1
            else:
                error = np.max(
                    np.abs(
                        observation.values[f"robot/{self.side}/joint_position"]
                        - self.command[self.arm_slice]
                    )
                )
                self.stable = self.stable + 1 if error < 0.01 else 0
                if self.stable >= 3:
                    self._advance(events)
        if self.stage != "wait" and self.stage_steps * self.dt > 18:
            raise SourceFailure(f"Coin stage {self.stage} timed out", kind="skill")
        self.stage_steps += 1
        self.step += 1
        return Action(self.command.copy(), tuple(events))
