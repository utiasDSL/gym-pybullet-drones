import os
import numpy as np
import torch
import torch.nn as nn
from gym.spaces import Box, Dict
from ray.rllib.models.torch.fcnet import FullyConnectedNetwork
from ray.rllib.agents.callbacks import DefaultCallbacks
from ray.rllib.models.torch.torch_modelv2 import TorchModelV2
from ray.rllib.models import ModelCatalog
from ray.rllib.policy.sample_batch import SampleBatch

OWN_OBS_VEC_SIZE = None
ACTION_VEC_SIZE = None

######################################################################################################################################################
def read_file_inputs():
    # This is a hack around a circular dependency and it should be removed
    OBS_FILE = os.path.dirname(os.path.abspath(__file__))+'/results/obs.txt'
    ACT_FILE = os.path.dirname(os.path.abspath(__file__))+'/results/act.txt'
    global OWN_OBS_VEC_SIZE, ACTION_VEC_SIZE
    if OWN_OBS_VEC_SIZE is None: 
        with open(OBS_FILE, 'r+') as f: OWN_OBS_VEC_SIZE = int(f.read())
        os.remove(OBS_FILE)
    if ACTION_VEC_SIZE is None: 
        with open(ACT_FILE, 'r+') as f: ACTION_VEC_SIZE = int(f.read())
        os.remove(ACT_FILE)

######################################################################################################################################################
class CustomTorchCentralizedCriticModel(TorchModelV2, nn.Module):
    """Multi-agent model that implements a centralized value function.

    It assumes the observation is a dict with 'own_obs' and 'opponent_obs', the
    former of which can be used for computing actions (i.e., decentralized
    execution), and the latter for optimization (i.e., centralized learning).

    This model has two parts:
    - An action model that looks at just 'own_obs' to compute actions
    - A value model that also looks at the 'opponent_obs' / 'opponent_action'
      to compute the value (it does this by using the 'obs_flat' tensor).
    """

    def __init__(self, obs_space, action_space, num_outputs, model_config, name):
        read_file_inputs()
        TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        nn.Module.__init__(self)
        self.action_model = FullyConnectedNetwork(
            Box(low=-1, high=1, shape=(OWN_OBS_VEC_SIZE, )), 
            action_space,
            num_outputs,
            model_config,
            name + "_action"
            )
        self.value_model = FullyConnectedNetwork(
            obs_space, 
            action_space,
            1, 
            model_config, 
            name + "_vf"
            )
        self._model_in = None

    def forward(self, input_dict, state, seq_lens):
        self._model_in = [input_dict["obs_flat"], state, seq_lens]
        return self.action_model({ "obs": input_dict["obs"]["own_obs"] }, state, seq_lens)

    def value_function(self):
        value_out, _ = self.value_model({ "obs": self._model_in[0] }, self._model_in[1], self._model_in[2])
        return torch.reshape(value_out, [-1])

######################################################################################################################################################
class FillInActions(DefaultCallbacks):
    def on_postprocess_trajectory(self, worker, episode, agent_id, policy_id, policies, postprocessed_batch, original_batches, **kwargs):
        read_file_inputs()
        to_update = postprocessed_batch[SampleBatch.CUR_OBS]
        other_id = 1 if agent_id == 0 else 0
        action_encoder = ModelCatalog.get_preprocessor_for_space( 
                                                        Box(-np.inf, np.inf, (ACTION_VEC_SIZE,), np.float32) # Unbounded
                                                        )
        _, opponent_batch = original_batches[other_id]
        opponent_actions = np.array([ action_encoder.transform(a) for a in opponent_batch[SampleBatch.ACTIONS] ])
        to_update[:, -ACTION_VEC_SIZE:] = opponent_actions

######################################################################################################################################################
def central_critic_observer(agent_obs, **kw):
    read_file_inputs()
    new_obs = {
        0: {
            "own_obs": agent_obs[0],
            "opponent_obs": agent_obs[1],
            "opponent_action": np.zeros(ACTION_VEC_SIZE),  # Filled in by FillInActions
        },
        1: {
            "own_obs": agent_obs[1],
            "opponent_obs": agent_obs[0],
            "opponent_action": np.zeros(ACTION_VEC_SIZE),  # Filled in by FillInActions
        },
    }
    return new_obs








