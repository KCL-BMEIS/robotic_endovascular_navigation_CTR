from .state import State
from ..target import Target as TargetClass
from typing import Dict
import numpy as np


class Target(State):
    def __init__(
        self,
        target: TargetClass,
        name: str = "target",
    ) -> None:
        super().__init__(name)
        self.target = target

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = self.target.high
        return {self.name: np.array(high, dtype=np.float32)}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        low = self.target.low
        return {self.name: np.array(low, dtype=np.float32)}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high_episode = self.target.high_episode
        return {self.name: np.array(high_episode, dtype=np.float32)}

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        low_episode = self.target.low_episode
        return {self.name: np.array(low_episode, dtype=np.float32)}

    def step(self) -> None:
        self._state = np.array(self.target.coordinates, dtype=np.float32)

    def reset(self, episode_nr: int = 0) -> None:
        self._state = np.array(self.target.coordinates, dtype=np.float32)
