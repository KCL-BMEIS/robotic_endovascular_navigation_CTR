from typing import List, Tuple
import numpy as np
import torch
from .state import State
from .intervention import Intervention
from .model import SACModel
from .target import Target


class Controller:
    def __init__(
        self,
        intervention: Intervention,
        state: State,
        nn_model: SACModel,
        target: Target,
    ) -> None:
        self.state = state
        self.nn_model = nn_model
        self.intervention = intervention
        self.target = target

    def step(
        self,
        tracking: np.ndarray,
        target: np.ndarray,
        device_lengths_inserted: List[float],
    ) -> np.ndarray:
        self.target.coordinates = target
        self.intervention.tracking = tracking
        self.intervention.device_lengths_inserted = device_lengths_inserted

        self.state.step()
        state = self.state.state
        state = self.state.to_flat_state(state)
        action = self.nn_model.get_play_action(state, evaluation=True)
        action *= self.intervention.action_high.reshape(-1)
        self.intervention.last_action = action.reshape(
            self.intervention.last_action.shape
        )
        return action

    def reset(
        self,
        tracking: np.ndarray,
        target: np.ndarray,
        device_lengths_inserted: List[float],
        tracking_high_episode: Tuple[float, float, float] = None,
        tracking_low_episode: Tuple[float, float, float] = None,
    ):
        if tracking_high_episode is not None:
            self.intervention.tracking_high_episode = tracking_high_episode
            self.target.high_episode = tracking_high_episode
        if tracking_low_episode is not None:
            self.intervention.tracking_low_episode = tracking_low_episode
            self.target.low_episode = tracking_low_episode
        self.target.coordinates = target
        self.intervention.tracking = tracking
        self.intervention.device_lengths_inserted = device_lengths_inserted
        self.intervention.last_action = self.intervention.action_high * 0.0
        self.state.reset()
        self.nn_model.reset()
        self.nn_model.to(torch.device("cpu"))
