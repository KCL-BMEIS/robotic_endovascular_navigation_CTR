from ..state import State
from ...intervention import Intervention
import numpy as np
from typing import Dict, Optional


class RelativeToTip(State):
    def __init__(
        self,
        intervention: Intervention,
        wrapped_state: State,
        name: Optional[str] = None,
    ) -> None:
        name = name or wrapped_state.name
        super().__init__(name)
        if wrapped_state.shape[-1] == 3:
            raise ValueError(
                f"{self.__class__} can only be used with 3 dimensional States. Not with {wrapped_state.shape[-1]} Dimensions"
            )
        self.intervention = intervention
        self.wrapped_state = wrapped_state
        self._state = None

    @property
    def state(self) -> Dict[str, np.ndarray]:
        return {self.name: self._state}

    @property
    def high(self) -> Dict[str, np.ndarray]:
        high = self.wrapped_state.high[self.wrapped_state.name]
        low = self.intervention.tracking_low
        low = np.full_like(high, low, dtype=np.float32)
        high -= low
        return {self.name: high}

    @property
    def low(self) -> Dict[str, np.ndarray]:

        low = self.wrapped_state.low[self.wrapped_state.name]
        high = self.intervention.tracking_high
        high = np.full_like(low, high, dtype=np.float32)
        low -= high
        return {self.name: low}

    @property
    def high_episode(self) -> Dict[str, np.ndarray]:
        high_episode = self.wrapped_state.high_episode[self.wrapped_state.name]
        low_episode = self.intervention.tracking_low_episode
        low_episode = np.full_like(high_episode, low_episode, dtype=np.float32)
        high_episode -= low_episode
        return {self.name: high_episode}

    @property
    def low_episode(self) -> Dict[str, np.ndarray]:
        low_episode = self.wrapped_state.low_episode[self.wrapped_state.name]
        high_episode = self.intervention.tracking_high_episode
        high_episode = np.full(low_episode, high_episode, dtype=np.float32)
        low_episode -= high_episode
        return {self.name: low_episode}

    def step(self) -> None:
        self.wrapped_state.step()
        self._calc_state()

    def reset(self, episode_nr: int = 0) -> None:
        self.wrapped_state.reset(episode_nr)
        self._calc_state()

    def _calc_state(self):
        state = self.wrapped_state.state[self.wrapped_state.name]
        tip = self.intervention.tracking[0]
        subtrahend = np.full_like(state, tip)
        self._state = state - subtrahend
