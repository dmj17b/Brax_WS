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
        sys = mjcf.load(self.filepath)
        # Defining time variables
        self._dt = 0.02
        n_frames = kwargs.pop('n_frames', int(self._dt/sys.opt.timestep))
        # Super init to pipeline env
        super().__init__(sys, backend='mjx', n_frames = n_frames)

        # Setting the initial state
        self._init_q = jnp.array(sys.mj_model.keyframe('home').qpos)
        self._default_ctrl = jnp.array(sys.mj_model.keyframe('home').ctrl)


    def reset(self) -> State:
        # Grab states, observations, etc. from the pipeline
        pipeline_state = self.pipeline_init(self._init_q, jnp.zeros(self.sys.qd_size()))
        observation = self.get_obs(pipeline_state)
        reward,done = jnp.zeros(2)
        metrics = {
            'rewards': reward,
            'observation': observation,
        }

        # Create the reset state object
        state = State(
            pipeline_state=pipeline_state,
            obs=observation,
            reward=reward,
            done=done,
            metrics=metrics,
        )

        return state


    # Function to define how a simulation step is done
    def step(self, state: State, action: jax.Array)->State:
        
        # Perform a forward physics step
        pipeline_state = self.pipeline_step(state.pipeline_state, action)

        rewards = {
            'vel_target': (0),
            'ang_vel_target': (0),
            'ride_height_target': -((0.2-pipeline_state.q[2])**2),
            'roll_target': (0),
            'pitch_target': (0),
        }
        reward = jnp.sum(jnp.array(list(rewards.values())))
        
        # Get Observation from new state:
        observation = self.get_obs(pipeline_state)
        state = state.replace(
            pipeline_state=pipeline_state,
            obs=observation,
            reward=reward,
        )
        return state


    # Gets the current observation from the environment
    def get_obs(
            self,
            pipeline_state: State,
            ) -> jax.Array:
        
        obs = jnp.concatenate([
            pipeline_state.q,
            pipeline_state.qd,
        ])
        return obs
        
        
envs.register_environment('walter', WalterEnv)

# Dummy test script:

def test():
    env = WalterEnv()
    state = env.reset()
    env.step(state, env._default_ctrl)


if __name__ == '__main__':
    test()