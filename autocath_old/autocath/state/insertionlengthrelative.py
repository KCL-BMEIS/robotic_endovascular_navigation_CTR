from ..intervention import Intervention
from .state import State
import numpy as np
from typing import Dict


class InsertionLengthRelative(State):
    def __init__(
        self,
        intervention: Intervention,
        device_id: int,
        relative_to_device_id: int,
        name: str = None,
    ) -> None:
        name = (
            name
            or f"device_{device_id}_length_relative_to_device_{relative_to_device_id}"
        )
        super().__init__(name)
        self.intervention = intervention
        self.device_id = device_id
        self.relative_to_device_id = relative_to_device_id
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = self.intervention.device_lengths_maximum[self.device_id]
        return {self.name: np.array(high, dtype=np.float32)}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        low = -self.intervention.device_lengths_maximum[self.relative_to_device_id]
        return {self.name: np.array(low, dtype=np.float32)}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        return self.high

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        return self.low

    def step(self) -> None:
        inserted_lengths = self.intervention.device_lengths_inserted
        self._state = np.array(
            inserted_lengths[self.device_id]
            - inserted_lengths[self.relative_to_device_id],
            dtype=np.float32,
        )

    def reset(self, episode_nr: int = 0) -> None:
        self.step()
