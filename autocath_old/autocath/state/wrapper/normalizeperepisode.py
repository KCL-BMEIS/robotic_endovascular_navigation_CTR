from .normalize import Normalize
import numpy as np


class NormalizePerEpisode(Normalize):
    def _normalize(self, state) -> np.ndarray:
        low = self.wrapped_state.low_episode[self.wrapped_state.name]
        high = self.wrapped_state.high_episode[self.wrapped_state.name]
        return np.array(2 * ((state - low) / (high - low)) - 1, dtype=np.float32)
