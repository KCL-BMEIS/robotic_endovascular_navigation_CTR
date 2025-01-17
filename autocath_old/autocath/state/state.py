from abc import ABC, abstractmethod
from typing import Dict, Tuple
import numpy as np


class State(ABC):
    def __init__(self, name: str) -> None:
        self.name = name

    @property
    @abstractmethod
    def state(self) -> Dict[str, np.ndarray]:
        ...

    @property
    @abstractmethod
    def high(self) -> Dict[str, np.ndarray]:
        ...

    @property
    @abstractmethod
    def low(self) -> Dict[str, np.ndarray]:
        ...

    @property
    @abstractmethod
    def high_episode(self) -> Dict[str, np.ndarray]:
        ...

    @property
    @abstractmethod
    def low_episode(self) -> Dict[str, np.ndarray]:
        ...

    @abstractmethod
    def step(self) -> None:
        ...

    @abstractmethod
    def reset(self, episode_nr: int = 0) -> None:
        ...

    @property
    def shape(self) -> Dict[str, Tuple]:
        state = self.low
        shape = {}
        for key in state.keys():
            shape.update({key: state[key].shape})
        return shape

    @staticmethod
    def to_flat_state(state: Dict[str, np.ndarray]) -> np.ndarray:
        keys = tuple(sorted(state.keys()))
        flat_state = np.array([], dtype=np.float32)
        for key in keys:
            new_state = state[key]
            new_state = np.array(new_state)
            if new_state.shape:
                new_state = new_state.reshape((-1,))
            flat_state = np.append(flat_state, new_state)
        return flat_state
