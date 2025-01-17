from ..intervention import Intervention
from .state import State
import numpy as np
from typing import Dict, List


class Tracking(State):
    def __init__(
        self,
        intervention: Intervention,
        n_points: int = 2,
        resolution: float = 1.0,
        name: str = "tracking",
    ) -> None:
        super().__init__(name)
        self.intervention = intervention
        self.n_points = n_points
        self.resolution = resolution
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = [self.intervention.tracking_high] * self.n_points
        return {self.name: np.array(high, dtype=np.float32)}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        low = [self.intervention.tracking_low] * self.n_points
        return {self.name: np.array(low, dtype=np.float32)}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high_episode = [self.intervention.tracking_high_episode] * self.n_points
        return {self.name: np.array(high_episode, dtype=np.float32)}

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        low_episode = [self.intervention.tracking_low_episode] * self.n_points
        return {self.name: np.array(low_episode, dtype=np.float32)}

    def step(self) -> None:
        self._state = self._calculate_tracking_state()

    def reset(self, episode_nr: int = 0) -> None:
        self._state = self._calculate_tracking_state()

    def _calculate_tracking_state(self) -> np.ndarray:
        tracking = self.intervention.tracking
        inserted_length = max(self.intervention.device_lengths_inserted)
        tracking_state = self._evenly_distributed_tracking(tracking, inserted_length)

        return np.array(tracking_state, dtype=np.float32)

    def _evenly_distributed_tracking(self, tracking: np.ndarray, inserted_length):

        tracking_diff = tracking[:-1] - tracking[1:]
        tracking_length = np.linalg.norm(tracking_diff, axis=-1)
        tracking_length = np.sum(tracking_length)
        if tracking_length > 0 and inserted_length > 0:
            scaled_resolution = tracking_length / inserted_length * self.resolution
        else:
            scaled_resolution = self.resolution
        tracking = list(tracking)
        tracking_state = [tracking[0]]
        if self.n_points > 1:
            acc_dist = 0.0
            for point, next_point in zip(tracking[1:], tracking[:-1]):
                if len(tracking_state) >= self.n_points or np.all(point == next_point):
                    break
                length = np.linalg.norm(next_point - point)
                acc_dist += length
                while (
                    acc_dist >= scaled_resolution
                    and len(tracking_state) < self.n_points
                ):
                    unit_vector = (next_point - point) / length
                    tracking_point = next_point - unit_vector * (
                        acc_dist - scaled_resolution
                    )
                    tracking_state.append(tracking_point)
                    acc_dist -= scaled_resolution

            while len(tracking_state) < self.n_points:
                tracking_state.append(tracking_state[-1])
        return tracking_state
