from .state import State
from ..intervention import Intervention
from typing import Dict
import numpy as np


class LastAction(State):
    def __init__(self, intervention: Intervention, name: str = "last_action") -> None:
        super().__init__(name)
        self.intervention = intervention
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state.astype(np.float32)}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        return {self.name: self.intervention.action_high.astype(np.float32)}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        return {self.name: self.intervention.action_low.astype(np.float32)}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        return self.high

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        return self.low

    def step(self) -> None:
        self._state = self.intervention.last_action

    def reset(self, episode_nr: int = 0) -> None:
        self._state = self.intervention.last_action * 0.0
