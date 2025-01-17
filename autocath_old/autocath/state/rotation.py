from .state import State
from ..intervention import Intervention
from typing import Dict

import numpy as np

from math import sin, cos


class Rotation(State):
    def __init__(self, intervention: Intervention, name: str = "rotation") -> None:
        super().__init__(name)
        self.intervention = intervention
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        n_rotations = len(self.intervention.device_rotations)
        shape = (n_rotations, 2)
        return {self.name: np.ones(shape, dtype=np.float32)}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        n_rotations = len(self.intervention.device_rotations)
        shape = (n_rotations, 2)
        return {self.name: -np.ones(shape, dtype=np.float32)}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        return self.high

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        return self.low

    def step(self) -> None:
        rotation_data = self.intervention.device_rotations
        state = []
        for rotation in rotation_data:
            state.append([sin(rotation), cos(rotation)])

        self._state = np.array(state, dtype=np.float32)

    def reset(self, episode_nr: int = 0) -> None:
        self.step()
