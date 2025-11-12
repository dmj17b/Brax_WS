import time
import mujoco
import mujoco.viewer
import numpy as np
import pygame
import lib.MotorModel as motor

class RandomController:
    def __init__(self, m: mujoco.MjModel, d: mujoco.MjData, motors: motor.MotorModel):
        self.m = m
        self.d = d
        self.motors = motors


        # User control variables:
        self.joystick_deadzone = 0.2
        self.max_wheel_vel = 20
        self.max_knee_vel = 0.01

        # Desired setpoints
        self.fr_knee_des_pos = 0
        self.fl_knee_des_pos = 0
        self.br_knee_des_pos = 0
        self.bl_knee_des_pos = 0

        self.fr_knee_des_vel = 0
        self.fl_knee_des_vel = 0
        self.br_knee_des_vel = 0
        self.bl_knee_des_vel = 0

        self.fr_hip_des_pos = 0
        self.fl_hip_des_pos = 0
        self.br_hip_des_pos = 0
        self.bl_hip_des_pos = 0

        self.fr_hip_des_vel = 0
        self.fl_hip_des_vel = 0
        self.br_hip_des_vel = 0
        self.bl_hip_des_vel = 0

        self.right_wheel_vel_des = 0
        self.left_wheel_vel_des = 0

        self.d_up = 0
        self.d_down = 0
        self.d_left = 0
        self.d_right = 0

        self.hip_splay = 0
        self.dt = m.opt.timestep

        self.random_hip_amplitude = 15
        self.random_knee_amplitude = 150
        self.random_wheel_amplitude = 200
        self.random_phase_offset = 0.5

    def control(self, m: mujoco.MjModel, d: mujoco.MjData):

        # Calculate random 
        self.fr_hip_des_pos = np.random.uniform(-self.random_hip_amplitude, self.random_hip_amplitude)*np.sin(time.time()/2)
        self.fr_hip_des_pos = np.clip(self.fr_hip_des_pos, -0.5, 0.5)
        self.fl_hip_des_pos = np.random.uniform(-self.random_hip_amplitude, self.random_hip_amplitude)*np.sin(time.time()/2)
        self.fl_hip_des_pos = np.clip(self.fl_hip_des_pos, -0.5, 0.5)
        self.br_hip_des_pos = np.random.uniform(-self.random_hip_amplitude, self.random_hip_amplitude)*np.sin(time.time()/2)
        self.br_hip_des_pos = np.clip(self.br_hip_des_pos, -0.5, 0.5)
        self.bl_hip_des_pos = np.random.uniform(-self.random_hip_amplitude, self.random_hip_amplitude)*np.sin(time.time()/2)
        self.bl_hip_des_pos = np.clip(self.bl_hip_des_pos, -0.5, 0.5)

        self.fr_knee_des_pos += np.random.uniform(-self.random_knee_amplitude, self.random_knee_amplitude)*np.sin(time.time()/6)*self.dt
        self.fl_knee_des_pos += np.random.uniform(-self.random_knee_amplitude, self.random_knee_amplitude)*np.sin(time.time()/6)*self.dt
        self.br_knee_des_pos += np.random.uniform(-self.random_knee_amplitude, self.random_knee_amplitude)*np.sin(time.time()/6)*self.dt
        self.bl_knee_des_pos += np.random.uniform(-self.random_knee_amplitude, self.random_knee_amplitude)*np.sin(time.time()/6)*self.dt

        self.wheel1_des_vel = np.random.uniform(-self.random_wheel_amplitude, self.random_wheel_amplitude)*np.sin(time.time()/20)
        self.wheel2_des_vel = np.random.uniform(-self.random_wheel_amplitude, self.random_wheel_amplitude)*np.sin(time.time()/20)
        self.wheel3_des_vel = np.random.uniform(-self.random_wheel_amplitude, self.random_wheel_amplitude)*np.sin(time.time()/20)
        self.wheel4_des_vel = np.random.uniform(-self.random_wheel_amplitude, self.random_wheel_amplitude)*np.sin(time.time()/20)
        self.wheel5_des_vel = np.random.uniform(-self.random_wheel_amplitude, self.random_wheel_amplitude)*np.sin(time.time()/20)
        self.wheel6_des_vel = np.random.uniform(-self.random_wheel_amplitude, self.random_wheel_amplitude)*np.sin(time.time()/20)
        self.wheel7_vel = np.random.uniform(-self.random_wheel_amplitude, self.random_wheel_amplitude)*np.sin(time.time()/20)

        if self.check_health() == False:
            print("Robot flipped over! Resetting position.")
            mujoco.mj_resetData(m, d)

        self.send_commands()


    # Function that sends the commands to the motors
    def send_commands(self):

        self.motors[0].pos_control(self.fr_hip_des_pos)
        self.motors[1].pos_control(self.fl_hip_des_pos)
        self.motors[2].pos_control(self.br_hip_des_pos)
        self.motors[3].pos_control(self.bl_hip_des_pos)

        self.motors[4].pos_control(self.fr_knee_des_pos)
        self.motors[5].pos_control(self.fl_knee_des_pos)
        self.motors[6].pos_control(self.br_knee_des_pos)
        self.motors[7].pos_control(self.bl_knee_des_pos)

        self.motors[9].vel_control(self.wheel1_des_vel)
        self.motors[8].vel_control(self.wheel2_des_vel)
        self.motors[10].vel_control(self.wheel3_des_vel)
        self.motors[11].vel_control(self.wheel4_des_vel)
        self.motors[12].vel_control(self.wheel5_des_vel)
        self.motors[13].vel_control(self.wheel6_des_vel)
        self.motors[14].vel_control(self.wheel7_des_vel)
        self.motors[15].vel_control(self.wheel8_des_vel)

    def check_health(self):
        # Check if robot is flipped over (z position of torso)
        torso_z = self.d.xpos[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'torso'), 2]
        if torso_z < 0.1:
            return False
        return True