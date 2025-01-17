from abc import ABC
from typing import Dict, List
import numpy as np

from .state import State


class Combination(State, ABC):
    def __init__(self, states: List[State], name: str = "state_combination") -> None:
        super().__init__(name)
        self.states = states
        self._state = {}

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return self._state

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = {}
        for wrapped_state in self.states:
            high.update(wrapped_state.high)
        return high

    @property
    def low(self) -> Dict[str, np.ndarray]:
        low = {}
        for wrapped_state in self.states:
            low.update(wrapped_state.low)
        return low

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high = {}
        for wrapped_state in self.states:
            high.update(wrapped_state.high_episode)
        return high

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        low = {}
        for wrapped_state in self.states:
            low.update(wrapped_state.low_episode)
        return low

    def step(self) -> None:
        self._state = {}
        for wrapped_state in self.states:
            wrapped_state.step()
            self._state.update(wrapped_state.state)

    def reset(self, episode_nr: int = 0) -> None:
        self._state = {}
        for wrapped_state in self.states:
            wrapped_state.reset(episode_nr)
            self._state.update(wrapped_state.state)
