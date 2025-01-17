from ..state import State
from . import MemoryResetMode
from typing import Dict, Optional
import numpy as np


class Memory(State):
    def __init__(
        self,
        wrapped_state: State,
        n_steps: int,
        reset_mode: MemoryResetMode,
        name: Optional[str] = None,
    ) -> None:
        name = name or wrapped_state.name
        super().__init__(name)
        assert reset_mode in [
            0,
            1,
        ], f"Reset mode must be 'MemoryResetMode.FILL' or 'MemoryResetMode.ZERO'. {reset_mode} is not possible"
        self.wrapped_state = wrapped_state
        self.n_steps = n_steps
        self.reset_mode = reset_mode
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = self.wrapped_state.high[self.wrapped_state.name]
        high = np.repeat([high], self.n_steps, axis=0)
        return {self.name: high}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        low = self.wrapped_state.low[self.wrapped_state.name]
        low = np.repeat([low], self.n_steps, axis=0)
        return {self.name: low}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high_episode = self.wrapped_state.high_episode[self.wrapped_state.name]
        high_episode = np.repeat([high_episode], self.n_steps, axis=0)
        return {self.name: high_episode}

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        low_episode = self.wrapped_state.low_episode[self.wrapped_state.name]
        low_episode = np.repeat([low_episode], self.n_steps, axis=0)
        return {self.name: low_episode}

    def step(self) -> None:
        self.wrapped_state.step()
        wrapped_state = self.wrapped_state.state[self.wrapped_state.name]
        self._state[1:] = self._state[:-1]
        self._state[0] = wrapped_state

    def reset(self, episode_nr: int = 0) -> None:
        self.wrapped_state.reset(episode_nr)
        wrapped_state = self.wrapped_state.state[self.wrapped_state.name]
        if self.reset_mode == MemoryResetMode.FILL:
            self._state = np.repeat([wrapped_state], self.n_steps, axis=0)
        else:
            state = np.repeat([wrapped_state], self.n_steps, axis=0) * 0.0
            state[0] = wrapped_state
            self._state = state
