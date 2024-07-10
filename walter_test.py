from typing import Union

import os
from absl import app
import time

import numpy as np

import mujoco
import mujoco.viewer

import matplotlib.pyplot as plt


def position_controller(
    mj_data: mujoco.MjData,
    desired_position: np.ndarray,
    desired_velocity: Union[np.ndarray, None] = None,
    desired_torque: Union[np.ndarray, None] = None,
) -> np.ndarray:
    def wrap_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))
    # ODrive Controller:
    kp = 20.0
    kv = 1.0 / 6.0
    velocity_limit = 2.0
    torque_limit = 0.1

    if desired_velocity is None:
        desired_velocity = np.zeros_like(desired_position)
    if desired_torque is None:
        desired_torque = np.zeros_like(desired_position)

    position_error = desired_position - mj_data.qpos

    desired_velocity += kp * position_error
    desired_velocity = np.clip(
        desired_velocity, -velocity_limit, velocity_limit,
    )  # type: ignore
    velocity_error = desired_velocity - mj_data.qvel

    desired_torque += kv * velocity_error
    desired_torque = np.clip(
        desired_torque, -torque_limit, torque_limit,
    )  # type: ignore

    return desired_torque  # type: ignore


def pd_controller(
    mj_data: mujoco.MjData,
    desired_position: np.ndarray,
    desired_velocity: Union[np.ndarray, None] = None,
) -> np.ndarray:
    kp = 10.0
    kd = 2.0
    torque_limit = 0.1

    if desired_velocity is None:
        desired_velocity = np.zeros_like(desired_position)

    position_error = desired_position - mj_data.qpos
    velocity_error = desired_velocity - mj_data.qvel
    torque = kp * position_error + kd * velocity_error
    torque = np.clip(torque, -torque_limit, torque_limit)

    return torque


def main(argv=None):
    filepath = os.path.join(os.path.dirname(__file__), 'models/walter_leg/scene.xml')
    mj_model = mujoco.MjModel.from_xml_path(filepath)
    mj_data = mujoco.MjData(mj_model)

    # Initialize mj_model to Keyframe:
    mj_data.qpos = mj_model.keyframe('home').qpos
    mj_data.ctrl = mj_model.keyframe('home').ctrl

    # Load Replay Data:
    odrive_data = np.loadtxt('test_1.csv', delimiter=',')
    time_vector = odrive_data[:, 0]
    knee_torque = odrive_data[:, 1]
    knee_position = odrive_data[:, 2]
    knee_velocity = odrive_data[:, 3]
    hip_torque = odrive_data[:, 4]
    hip_position = odrive_data[:, 5]
    hip_velocity = odrive_data[:, 6]

    control_rate = 0.02
    num_steps = int(control_rate / mj_model.opt.timestep)

    terminal_condition = False
    position_history = []
    # with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
    while not terminal_condition:
        for i in range(len(time_vector)):
            step_time = time.time()
            # mj_data.ctrl = np.array([hip_torque[i], knee_torque[i]])
            # ctrl = position_controller(
            #     mj_data, np.array([hip_position[i], knee_position[i]]),
            # )
            # mj_data.ctrl = ctrl
            ctrl = pd_controller(
                mj_data, np.array([hip_position[i], knee_position[i]]),
            )
            mj_data.ctrl = ctrl
            position_history.append(mj_data.qpos.copy())

            for _ in range(num_steps):
                mujoco.mj_step(mj_model, mj_data)  # type: ignore

            # viewer.sync()

            sleep_time = control_rate - (time.time() - step_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

        terminal_condition = True

    position_history = np.asarray(position_history)


    fig, axs = plt.subplots(3, 1)
    axs[0].plot(position_history[:, 0], label='hip')
    axs[0].plot(hip_position, label='hip_ref')
    axs[0].legend()
    axs[1].plot(position_history[:, 1], label='knee')
    axs[1].plot(knee_position, label='knee_ref')
    axs[1].legend()
    axs[2].plot(knee_torque, label='knee')
    axs[2].plot(hip_torque, label='hip')
    axs[2].legend()

    fig.savefig('joint_positions.png')


if __name__ == '__main__':
    app.run(main)
