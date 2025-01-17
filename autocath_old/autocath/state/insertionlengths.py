from .state import State
from ..intervention import Intervention
from typing import Dict

import numpy as np



class InsertionLengths(State):
    def __init__(
        self, intervention: Intervention, name: str = "inserted_lengths"
    ) -> None:
        super().__init__(name)
        self.intervention = intervention
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = np.array(self.intervention.device_lengths_maximum, dtype=np.float32)
        return {self.name: high}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        n_devices = len(self.intervention.device_lengths_inserted)
        shape = (n_devices,)
        return {self.name: -np.zeros(shape, dtype=np.float32)}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        return self.high

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        return self.low

    def step(self) -> None:
        self._state = np.array(
            self.intervention.device_lengths_inserted, dtype=np.float32
        )

    def reset(self, episode_nr: int = 0) -> None:
        self.step()
