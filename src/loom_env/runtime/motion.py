"""Simulator-independent commands and measurements for embodiment diagnostics."""

import math

import numpy as np

from loom_env.specs.config import ARMS
from loom_env.specs.episode import Action, Event, Outcome, TaskStatus

DURATION = 12.0
PHASES = (
    "Right arm moves / left holds",
    "Left arm moves / right holds",
    "Both grippers close and open",
    "Both arms hold home",
)


def motion_phase(time):
    return 0 if time < 4 else 1 if time < 8 else 2 if time < 10 else 3


def gripper_command(gripper, position):
    """Recover a scalar command coordinate from the declared affine joint map."""
    matrix = np.asarray(gripper.joint_map)
    if matrix.shape[1] != 1 or np.linalg.matrix_rank(matrix) != 1:
        raise ValueError("Motion diagnostic requires one observable gripper command")
    return float(
        np.linalg.lstsq(
            matrix, position - np.asarray(gripper.joint_offset), rcond=None
        )[0][0]
    )


class MotionSource:
    def __init__(self, deployment):
        self.deployment = deployment
        self.home = np.concatenate(
            [
                np.r_[
                    deployment.arms[side].initial_positions,
                    [high for _, high in deployment.arms[side].gripper.command_limits],
                ]
                for side in ARMS
            ]
        )
        self.offsets = {}
        for side in ARMS:
            arm = deployment.arms[side]
            gripper_command(arm.gripper, np.asarray(arm.gripper.joint_offset))
            q = np.asarray(arm.initial_positions)
            limits = np.asarray(arm.joint_limits)
            sign = np.ones(len(q))
            sign[0] = 1 if side == "left" else -1
            desired = np.full(len(q), 0.15)
            desired[0] = 0.55
            for index in range(len(q)):
                if (
                    not limits[index, 0]
                    <= q[index] + sign[index] * desired[index]
                    <= limits[index, 1]
                ):
                    sign[index] *= -1
            self.offsets[side] = sign * desired
            peak = self.home.copy()
            peak[deployment.action_slices[f"{side}/arm"]] += self.offsets[side]
            deployment.validate_action(peak)

    def reset(self, episode_input):
        self.previous_phase = None

    def act(self, observation):
        time = observation.timestamp
        phase = motion_phase(time)
        command = self.home.copy()
        if phase < 2:
            side = "right" if phase == 0 else "left"
            amount = math.sin(math.pi * (time - phase * 4) / 4) ** 2
            command[self.deployment.action_slices[f"{side}/arm"]] += (
                self.offsets[side] * amount
            )
        elif phase == 2:
            amount = math.sin(math.pi * (time - 8) / 2) ** 2
            for side in ARMS:
                low, high = self.deployment.arms[side].gripper.command_limits[0]
                command[self.deployment.action_slices[f"{side}/gripper"]] = (
                    high - (high - low) * amount
                )
        events = ()
        if phase != self.previous_phase:
            events = (
                Event(
                    "skill",
                    PHASES[phase],
                    round(time / self.deployment.control_dt),
                    {"diagnostic": True, "phase": phase},
                ),
            )
            self.previous_phase = phase
        return Action(self.deployment.validate_action(command), events)


class MotionCheck:
    """Require every arm joint and each gripper to move, using measured state."""

    def __init__(self, deployment):
        self.deployment = deployment

    def reset(self, initial_state):
        self.excursion = {
            side: np.zeros(len(self.deployment.arms[side].joint_names)) for side in ARMS
        }
        self.hold_error = np.zeros(2)
        self.minimum = np.full(2, np.inf)
        self.maximum = np.full(2, -np.inf)

    def update(self, world_state, dt):
        time = float(world_state["diagnostic/time"])
        error = world_state["diagnostic/tracking_error"]
        for side in ARMS:
            self.excursion[side] = np.maximum(
                self.excursion[side], world_state[f"diagnostic/{side}/joint_excursion"]
            )
        command = world_state["diagnostic/gripper_command"]
        self.minimum = np.minimum(self.minimum, command)
        self.maximum = np.maximum(self.maximum, command)
        phase = motion_phase(time - dt / 2)
        if phase < 2:
            held = 0 if phase == 0 else 1
            self.hold_error[held] = max(self.hold_error[held], error[held])
        if time < DURATION - 1e-8:
            return TaskStatus()
        ranges = np.array(
            [
                arm.gripper.command_limits[0][1] - arm.gripper.command_limits[0][0]
                for arm in (self.deployment.arms[side] for side in ARMS)
            ]
        )
        travel = (self.maximum - self.minimum) / ranges
        self.report = {
            "arm_order": list(ARMS),
            "joint_excursion_rad": {
                side: v.tolist() for side, v in self.excursion.items()
            },
            "max_held_arm_error_rad": self.hold_error.tolist(),
            "final_tracking_error_rad": error.tolist(),
            "gripper_command_range": np.stack(
                [self.minimum, self.maximum], axis=1
            ).tolist(),
            "gripper_travel_fraction": travel.tolist(),
            "criteria": {
                "min_each_joint_excursion_rad": 0.08,
                "max_held_arm_error_rad": 0.04,
                "max_final_tracking_error_rad": 0.04,
                "min_gripper_travel_fraction": 0.8,
            },
        }
        passed = bool(
            all(np.all(v > 0.08) for v in self.excursion.values())
            and np.all(self.hold_error < 0.04)
            and np.all(error < 0.04)
            and np.all(travel > 0.8)
        )
        return TaskStatus(
            Outcome(
                "success" if passed else "task_failure",
                "joint_motion_diagnostic_passed"
                if passed
                else "joint_motion_threshold_failed",
            )
        )
