from typing import Any, List, Sequence
import os 
import jax
import jax.numpy as jnp
import numpy as np
from brax import base
from brax import envs
from brax import math
from brax.base import Motion, Transform
from brax.envs.base import PipelineEnv, State
from brax.io import mjcf, html
from brax.mjx import pipeline
from etils import epath
from ml_collections import config_dict
import mujoco


class WalterEnv(PipelineEnv):

    def __init__(self, filename: str = 'walter/scene.xml', **kwargs):
        # Load the MJCF file
        filename = f'models/{filename}'
        self.filepath = os.path.join(
            os.path.dirname(
                os.path.dirname(
                    os.path.dirname(__file__),
                ),
            ),
            filename,
        )
        # Create the mjcf system
        print(self.filepath)
        sys = mjcf.load(self.filepath)
        print(sys.nv)
        # Defining time variables
        self._dt = 0.02
        n_frames = kwargs.pop('n_frames', int(self._dt/self.sys.opt.timestep))
        # Super init to pipeline env
        super().__init__(sys, backend='mjx', n_frames = n_frames)

        # Setting the initial state
        self._init_q = jnp.array(sys.mj_model.keyframe('home').qpos)
        self._default_ctrl = jnp.array(sys.mj_model.keyframe('home').ctrl)


    # What does -> do?
    # Step needs current state because of immutability?
    # pipeline_state: Optional[base.State]
    # obs: jax.Array
    # reward: jax.Array
    # done: jax.Array
    # metrics: Dict[str, jax.Array] = struct.field(default_factory=dict)
    # info: Dict[str, Any] = struct.field(default_factory=dict)

    # Could use some clarification on states - when and where each state gets used
    # What is the difference between pipeline_state and state?
    # Assumption: environment observation != mujoco state bc you may want more or
    # less info than mujoco state

    # Function to define how a simulation step is done
    def step(self, state: State, action: jax.Array)->State:
        
        # Perform a forward physics step
        pipeline_state = self.pipeline_step(state.pipeline_state, action)

        # Get Observation from new state:
        observation = self.get_obs(pipeline_state)

        # Get reward from new state:
        print(pipeline_state)


    # Function that defines how the environment is reset with each
    # new episode
    def reset(self):

        ...
    def get_obs(
            self,
            mj_data: mujoco.MjData,
            command: np.ndarray,
            previous_action: np.ndarray,):
        ...
        
        
envs.register_environment('walter', WalterEnv)

# Dummy test script:

def test():
    env = WalterEnv()
    env.step()


if __name__ == '__main__':
    test()