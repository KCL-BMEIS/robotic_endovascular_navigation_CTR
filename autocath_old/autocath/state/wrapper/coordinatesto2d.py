from ..state import State

from typing import Dict, Optional

import numpy as np


class CoordinatesTo2D(State):
    def __init__(
        self,
        wrapped_state: State,
        dimension_to_delete: str,
        name: Optional[str] = None,
    ) -> None:
        name = name or wrapped_state.name
        super().__init__(name)
        self.dimension_to_delete = dimension_to_delete
        if dimension_to_delete == "y":
            self._delete_idx = 1
        elif dimension_to_delete == "z":
            self._delete_idx = 2
        elif dimension_to_delete == "x":
            self._delete_idx = 0
        else:
            raise ValueError(
                f"{dimension_to_delete = } is invalid. Needs to be x,y or z"
            )
        self.wrapped_state = wrapped_state

        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = self.wrapped_state.high[self.wrapped_state.name]
        if high.shape[-1] == 3:
            high = np.delete(high, self._delete_idx, axis=-1)
        return {self.name: high}

    @property
    def low(self) -> Dict[str, np.ndarray]:
        low = self.wrapped_state.low[self.wrapped_state.name]
        if low.shape[-1] == 3:
            low = np.delete(low, self._delete_idx, axis=-1)
        return {self.name: low}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high_episode = self.wrapped_state.high_episode[self.wrapped_state.name]
        if high_episode.shape[-1] == 3:
            high_episode = np.delete(high_episode, self._delete_idx, axis=-1)
        return {self.name: high_episode}

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        low_episode = self.wrapped_state.low_episode[self.wrapped_state.name]
        if low_episode.shape[-1] == 3:
            low_episode = np.delete(low_episode, self._delete_idx, axis=-1)
        return {self.name: low_episode}

    def step(self) -> None:
        self.wrapped_state.step()
        state = self.wrapped_state.state[self.wrapped_state.name]
        if state.shape[-1] == 3:
            state = np.delete(state, self._delete_idx, axis=-1)
        self._state = state

    def reset(self, episode_nr: int = 0) -> None:
        self.wrapped_state.reset(episode_nr)
        state = self.wrapped_state.state[self.wrapped_state.name]
        if state.shape[-1] == 3:
            state = np.delete(state, self._delete_idx, axis=-1)
        self._state = state
