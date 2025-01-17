from typing import List, Tuple
import numpy as np


class Intervention:
    def __init__(
        self,
        tracking_high: Tuple[float, float, float],
        tracking_low: Tuple[float, float, float],
        device_lengths_maximum: List[float],
        velocity_limits: List[Tuple[float, float]],
    ) -> None:
        self.tracking_high = np.array(tracking_high)
        self.tracking_high_episode = self.tracking_high
        self.tracking_low = np.array(tracking_low)
        self.tracking_low_episode = self.tracking_low
        self.device_lengths_maximum = np.array(device_lengths_maximum)
        self.velocity_limits = np.array(velocity_limits)
        self.action_low = -self.velocity_limits
        self.action_high = self.velocity_limits
        self.tracking = None
        self.device_lengths_inserted = None
        self.last_action = None
