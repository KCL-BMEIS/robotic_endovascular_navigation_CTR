from .normalize import Normalize, State
from typing import  Optional
import numpy as np


class NormalizeCustom(Normalize):
    def __init__(
        self,
        wrapped_state: State,
        min_value: float,
        max_value: float,
        name: Optional[str] = None,
    ) -> None:
        super().__init__(wrapped_state, name)
        self.min_value = np.array(min_value, dtype=np.float32)
        self.max_value = np.array(max_value, dtype=np.float32)

    def _normalize(self, state) -> np.ndarray:
        low = self.min_value
        high = self.max_value
        return np.array(2 * ((state - low) / (high - low)) - 1, dtype=np.float32)
