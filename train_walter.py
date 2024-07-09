import functools
from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from src.envs.walter import WalterEnv

make_networks_factory = functools.partial(
    ppo_networks.make_ppo_networks,
    policy_hidden_layer_sizes = (256, 256),
)

train_fn = functools.partial(
    ppo.train,
    num_timesteps = 1000,
    num_evals = 1,
    episode_length = 20,
    num_envs = 1,
    num_eval_envs = 4,
    batch_size = 32,
    num_minibatches = 4,
    unroll_length = 20,
    num_updates_per_batch = 4,
    normalize_observations = True,
    discounting = 0.97,
    learning_rate = 3.0e-4,
    entropy_cost = 1e-2,
    network_factory = make_networks_factory,
    seed = 0,
)

env = WalterEnv()