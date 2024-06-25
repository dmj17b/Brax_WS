import os
from absl import app

import jax
import jax.numpy as jnp

from brax.mjx import pipeline
from brax.io import mjcf, html

def main(argv=None):
    # Load the MJCF file
    mjcf_path = os.path.join(os.path.dirname(__file__),
                             'models/walter/scene.xml')
    sys = mjcf.load(mjcf_path)

    # Create the pipeline
    # setting the initial state
    q_home = sys.mj_model.keyframe('home').qpos
    ctrl = sys.mj_model.keyframe('home').ctrl

    # Jitting the init and step functions for GPU acceleration
    init_fn = jax.jit(pipeline.init)
    step_fn = jax.jit(pipeline.step)

    # Calling the init function
    state = init_fn(sys=sys, q=q_home, qd=jnp.zeros(sys.qd_size()))

    # Running the simulation
    num_steps = 1000
    state_history = []
    for i in range(num_steps):
        state = step_fn(sys, state, act=ctrl)
        state_history.append(state)


    # # Visualizing the simulation
    html_string = html.render(
        sys = sys,
        states=state_history,
        height="100vh",
        colab=False)
    # Path for where to save the visualization
    html_path = os.path.join(
        os.path.dirname(__file__),
        "visualization/visualization.html",
    )

    with open(html_path, "w") as f:
        f.writelines(html_string)
    
if __name__ == '__main__':
    app.run(main)
