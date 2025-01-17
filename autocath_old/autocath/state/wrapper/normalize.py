from ..state import State
from typing import Dict, Optional
import numpy as np


class Normalize(State):
    def __init__(
        self,
        wrapped_state: State,
        name: Optional[str] = None,
    ) -> None:
        name = name or wrapped_state.name
        super().__init__(name)
        self.wrapped_state = wrapped_state
        self._state = None
        self._wrapped_low = None
        self._wrapped_high = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = self.wrapped_state.high[self.wrapped_state.name]
        high = self._normalize(high)
        return {self.name: high}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        low = self.wrapped_state.low[self.wrapped_state.name]
        low = self._normalize(low)
        return {self.name: low}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high_episode = self.wrapped_state.high_episode[self.wrapped_state.name]
        high_episode = self._normalize(high_episode)

        return {self.name: high_episode}

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        low_episode = self.wrapped_state.low_episode[self.wrapped_state.name]
        low_episode = self._normalize(low_episode)
        return {self.name: low_episode}

    def step(self) -> None:
        self.wrapped_state.step()
        wrapped_state = self.wrapped_state.state[self.wrapped_state.name]
        self._state = self._normalize(wrapped_state)

    def reset(self, episode_nr: int = 0) -> None:
        self.wrapped_state.reset(episode_nr)
        wrapped_state = self.wrapped_state.state[self.wrapped_state.name]
        self._state = self._normalize(wrapped_state)

    def _normalize(self, state) -> np.ndarray:
        low = self.wrapped_state.low[self.wrapped_state.name]
        high = self.wrapped_state.high[self.wrapped_state.name]
        return np.array(2 * ((state - low) / (high - low)) - 1, dtype=np.float32)
