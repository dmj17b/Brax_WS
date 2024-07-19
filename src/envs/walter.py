from typing import Any, Dict
import os
from absl import app

import flax.serialization
import jax
import jax.numpy as jnp

import flax

from brax import envs
from brax import math
from brax.envs.base import PipelineEnv, State
from brax.io import mjcf, html

# Types:
PRNGKey = jax.Array

@flax.struct.dataclass
class RewardConfig:
    # Rewards:
    tracking_linear_velocity: float = 1.5
    tracking_angular_velocity: float = 0.8
    # Penalties / Regularization Terms:
    linear_z_velocity: float = -2.0
    angular_xy_velocity: float = -0.05
    orientation: float = -5.0
    torque: float = -1e-4
    action_rate: float = -0.01
    stand_still: float = -0.5
    termination: float = -1.0
    slip: float = -0.1


class Walter(PipelineEnv):

    def __init__(
        self,
        filename: str = 'walter/scene.xml',
        config: RewardConfig = RewardConfig(),
        **kwargs,
    ):
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

        self.step_dt = 0.02
        n_frames = kwargs.pop('n_frames', int(self.step_dt/sys.opt.timestep))
        super().__init__(sys, backend='mjx', n_frames = n_frames)

        # Setting the initial state
        self._init_q = jnp.array(sys.mj_model.keyframe('home').qpos)
        self.default_pose = jnp.array(sys.mj_model.keyframe('home').qpos)[7:]
        self._default_ctrl = jnp.array(sys.mj_model.keyframe('home').ctrl)
        self.limb_idx = jnp.array([0, 1, 4, 5, 8, 9, 12, 13])
        self.wheel_idx = jnp.array([2, 3, 6, 7, 10, 11, 14, 15])

        # Set the reward config and other parameters:
        self.reward_weights = flax.serialization.to_state_dict(config)
        self.history_length = 15
        self.num_observations = 40

        # Control Scales:
        self._action_scale = 0.3
        self._torque_scale = 0.3


    def reset(self, rng: PRNGKey) -> State:
        # Generate new rng keys:
        key, cmd_key, q_key, qd_key = jax.random.split(rng, 4)

        q = self._init_q + jax.random.uniform(
            key=q_key,
            shape=(self.sys.q_size(),),
            minval=0.0,
            maxval=0.0,
        )

        qd = jax.random.uniform(
            key=qd_key,
            shape=(self.sys.qd_size(),),
            minval=0.0,
            maxval=0.0,
        )

        # Initialize the pipeline state:
        pipeline_state = self.pipeline_init(q=q, qd=qd)

        # Initialize Reward and Termination State:
        reward, done = jnp.zeros(2)
        done = jnp.float64(done) if jax.config.x64_enabled else jnp.float32(done)


        # State Info: (Used to track values inbetween steps)
        state_info = {
            'command': self.sample_command(cmd_key),
            'previous_state': {
                'q': q,
                'qd': qd,
            },
            'previous_action': jnp.zeros_like(self._default_ctrl),
            'rewards': {k: 0.0 for k in self.reward_weights.keys()},
        }

        # Metrics: (Reward Dictionary)
        metrics = {}
        for k in state_info['rewards']:
            metrics[k] =state_info['rewards'][k]

        # Initialize the observation:
        observation_history = jnp.zeros(
            self.history_length * self.num_observations,
        )
        observation = self.get_observation(pipeline_state, state_info, observation_history)

        # Create State:
        state = State(
            pipeline_state=pipeline_state,
            obs=observation,
            reward=reward,
            done=done,
            metrics=metrics,
            info=state_info,
        )

        return state


    # Function to define how a simulation step is done
    def step(self, state: State, action: jax.Array) -> State:
        
        # Perform a forward physics step
        position_targets = self._default_pose[self.limb_idx] + action[self.limb_idx] * self._action_scale
        torque_targets = action[self.wheel_idx] * self._torque_scale
        motor_targets = jnp.array([
            position_targets[0], position_targets[1], torque_targets[0], torque_targets[1],
            position_targets[2], position_targets[3], torque_targets[2], torque_targets[3],
            position_targets[4], position_targets[5], torque_targets[4], torque_targets[5],
            position_targets[6], position_targets[7], torque_targets[6], torque_targets[7],
        ])
        pipeline_state = self.pipeline_step(state.pipeline_state, motor_targets)

        reward = 0.0
        
        # Get New Observation:
        observation = self.get_observation(pipeline_state, state.info, state.obs)

        state = state.replace(
            pipeline_state=pipeline_state,
            obs=observation,
            reward=reward,
        )
        return state


    def sample_command(self, rng: PRNGKey) -> jnp.ndarray:
        lin_vel_x = [-0.6, 1.5]  # min max [m/s]
        lin_vel_y = [-0.8, 0.8]  # min max [m/s]
        ang_vel_yaw = [-0.7, 0.7]  # min max [rad/s]

        _, key1, key2, key3 = jax.random.split(rng, 4)
        lin_vel_x = jax.random.uniform(
            key1, (1,), minval=lin_vel_x[0], maxval=lin_vel_x[1]
        )
        lin_vel_y = jax.random.uniform(
            key2, (1,), minval=lin_vel_y[0], maxval=lin_vel_y[1]
        )
        ang_vel_yaw = jax.random.uniform(
            key3, (1,), minval=ang_vel_yaw[0], maxval=ang_vel_yaw[1]
        )
        new_cmd = jnp.array([lin_vel_x[0], lin_vel_y[0], ang_vel_yaw[0]])
        return new_cmd
    
    # Gets the current observation from the environment
    def get_observation(
        self,
        pipeline_state: State,
        state_info: Dict[str, Any],
        observation_history: jax.Array,
    ) -> jnp.ndarray:
        # Observation: [yaw_rate, projected_gravity, command, relative_motor_positions, last_action]
        inverse_base_rotation = math.quat_inv(
            pipeline_state.x.rot[0],
        )
        local_yaw_rate = math.rotate(
            pipeline_state.xd.ang[0],
            inverse_base_rotation,
        )[2]
        projected_gravity = math.rotate(
            jnp.array([0.0, 0.0, -1.0]),
            inverse_base_rotation,
        )

        observation = jnp.concatenate([
            jnp.array([local_yaw_rate]),
            projected_gravity,
            state_info['command'],
            pipeline_state.q[7:] - self.default_pose,
            state_info['previous_action'],
        ])
        observation = jnp.roll(observation_history, observation.size).at[:observation.size].set(observation)

        return observation
        
        
envs.register_environment('walter', Walter)

# Test Enviornment:
def main(argv=None):
    # RNG Key:
    key = jax.random.key(0)

    env = Walter()

    state = jax.jit(env.reset)(key)
    reset_fn = jax.jit(env.reset)
    step_fn = jax.jit(env.step)

    state = reset_fn(key)

    fwd_ctrl = jnp.array([
        0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, 0.5, 0.5,
    ])

    simulation_steps = 500
    state_history = []
    for i in range(simulation_steps):
        print(f"Step: {i}")
        state = step_fn(state, fwd_ctrl)
        state_history.append(state.pipeline_state)

    html_string = html.render(
        sys=env.sys.tree_replace({'opt.timestep': env.step_dt}),
        states=state_history,
        height="100vh",
        colab=False,
    )
    html_path = os.path.join(
        os.path.join(
            os.path.dirname(
                os.path.dirname(
                    os.path.dirname(__file__),
                ),
            ),
        ),
        "visualization/visualization.html",
    )

    with open(html_path, "w") as f:
        f.writelines(html_string)


if __name__ == '__main__':
    app.run(main)
