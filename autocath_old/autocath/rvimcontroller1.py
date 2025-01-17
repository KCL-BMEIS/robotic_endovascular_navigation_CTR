from typing import Tuple
import torch
from .controller import Controller, Target, Intervention, SACModel


from .confighandler import ConfigHandler


class RViMController1(Controller):
    def __init__(
        self,
        checkpoint: str,
        config: str,
        tracking_high: Tuple[float, float, float],
        tracking_low: Tuple[float, float, float],
    ) -> None:
        checkpoint = torch.load(checkpoint)
        self.config_handler = ConfigHandler()
        config = self.config_handler.load_config_dict(config)

        replace_to = self.__module__.split(".")[0]
        self.class_str_replace = [("eve", replace_to), ("stacierl", replace_to)]

        try:
            device = config["env_eval"]["intervention"]["device"]
            n_devices = 1
        except KeyError:
            device = config["env_eval"]["intervention"]["devices"][0]
            n_devices = len(config["env_eval"]["intervention"]["devices"])
        max_translation_velocity = device["velocity_limit"][0]
        max_rotation_velocity = device["velocity_limit"][1]
        self.n_devices = n_devices
        self.max_device_length = device["total_length"]
        intervention = Intervention(
            tracking_high,
            tracking_low,
            [self.max_device_length] * n_devices,
            [[max_translation_velocity, max_rotation_velocity]] * n_devices,
        )
        target = Target(tracking_high, tracking_low)

        state = self._create_state(config, intervention, target)
        nn_model = self._create_nn_model(config)

        network_states_container = nn_model.network_states_container
        network_states_container.from_dict(checkpoint["network_state_dicts"])
        nn_model.set_network_states(network_states_container)
        super().__init__(intervention, state, nn_model, target)

    def _create_state(self, config: dict, intervention: Intervention, target: Target):
        intervention_id = config["env_eval"]["intervention"]["_id"]
        target_id = config["env_eval"]["target"]["_id"]
        object_registry = {intervention_id: intervention, target_id: target}

        state_config = config["env_eval"]["state"]

        state = self.config_handler.config_dict_to_object(
            state_config, object_registry, self.class_str_replace
        )

        return state

    def _create_nn_model(self, config: dict) -> SACModel:
        model_config = config["algo"]["model"]

        model = self.config_handler.config_dict_to_object(
            model_config, class_str_replace=self.class_str_replace
        )
        return model
