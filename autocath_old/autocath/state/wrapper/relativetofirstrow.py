from ..state import State
import numpy as np
from typing import Dict, Optional


class RelativeToFirstRow(State):
    def __init__(
        self,
        wrapped_state: State,
        name: Optional[str] = None,
    ) -> None:
        name = name or wrapped_state.name
        super().__init__(name)
        self.wrapped_state = wrapped_state
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = self.wrapped_state.high[self.wrapped_state.name]
        wrapped_low = self.wrapped_state.low[self.wrapped_state.name]
        high[1:] = high[1:] - wrapped_low[1:]
        return {self.name: high}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        wrapped_high = self.wrapped_state.high[self.wrapped_state.name]
        low = self.wrapped_state.low[self.wrapped_state.name]
        low[1:] = low[1:] - wrapped_high[1:]
        return {self.name: low}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high_episode = self.wrapped_state.high_episode[self.wrapped_state.name]
        wrapped_low_episode = self.wrapped_state.low_episode[self.wrapped_state.name]
        high_episode[1:] = high_episode[1:] - wrapped_low_episode[1:]
        return {self.name: high_episode}

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        wrapped_high_episode = self.wrapped_state.high_episode[self.wrapped_state.name]
        low_episode = self.wrapped_state.low_episode[self.wrapped_state.name]
        low_episode[1:] = low_episode[1:] - wrapped_high_episode[1:]
        return {self.name: low_episode}

    def step(self) -> None:
        self.wrapped_state.step()
        self._calc_state()

    def reset(self, episode_nr: int = 0) -> None:
        self.wrapped_state.reset(episode_nr)
        self._calc_state()

    def _calc_state(self):
        state = self.wrapped_state.state[self.wrapped_state.name]
        subtrahend = np.full(state.shape, state[0])
        subtrahend[0] *= 0.0
        self._state = state - subtrahend
