import time
import mujoco
import mujoco.viewer
import numpy as np
import pygame
import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl

# Load in the model and data from xml file
m = mujoco.MjModel.from_xml_path('models/walter/scene.xml')
d = mujoco.MjData(m)

# Initializing motor model parameters:
hip_kv = 150
hip_voltage = 4*12  # 12 cell battery pack
knee_kv = 230
knee_voltage = 4*12 # 12 cell battery pack

hipParams = {
  'Kp': 400,
  'Kd': 80,
  'gear_ratio': 1,
  't_stall': 150,
  'w_no_load': hip_kv*hip_voltage*0.1047,
}

kneeParams = {
  'Kp': 100,
  'Kd': 30,
  'gear_ratio': 1,
  't_stall': 100,
  'w_no_load': knee_kv*knee_voltage*0.1047,
}

wheelParams = {
  'Kp': 0.1,
  'Kd': 0.05,
  'gear_ratio': 1,
  't_stall': 35,
  'w_no_load': 230*0.1047,
}

# Initializing motor models
fr_hip = motor.MotorModel(m, d, 'fr_hip', hipParams, 0)
fl_hip = motor.MotorModel(m, d,'fl_hip', hipParams, 4)
br_hip = motor.MotorModel(m, d,'br_hip', hipParams, 8)
bl_hip = motor.MotorModel(m, d,'bl_hip', hipParams, 12)

fr_knee = motor.MotorModel(m, d, 'fr_knee', kneeParams, 1)
fl_knee = motor.MotorModel(m, d, 'fl_knee', kneeParams, 5)
br_knee = motor.MotorModel(m, d, 'br_knee', kneeParams, 9)
bl_knee = motor.MotorModel(m, d, 'bl_knee', kneeParams, 13)

fr_wheel1_joint = motor.MotorModel(m, d, 'fr_wheel1_joint', wheelParams, 2)
fr_wheel2_joint = motor.MotorModel(m, d, 'fr_wheel2_joint', wheelParams, 3)
fl_wheel1_joint = motor.MotorModel(m, d, 'fl_wheel1_joint', wheelParams, 6)
fl_wheel2_joint = motor.MotorModel(m, d, 'fl_wheel2_joint', wheelParams, 7)
br_wheel1_joint = motor.MotorModel(m, d, 'br_wheel1_joint', wheelParams, 10)
br_wheel2_joint = motor.MotorModel(m, d, 'br_wheel2_joint', wheelParams, 11)
bl_wheel1_joint = motor.MotorModel(m, d, 'bl_wheel1_joint', wheelParams, 14)
bl_wheel2_joint = motor.MotorModel(m, d, 'bl_wheel2_joint', wheelParams, 15)
motors = [fr_hip, fl_hip, br_hip, bl_hip, 
          fr_knee, fl_knee, br_knee, bl_knee, 
          fr_wheel1_joint, fr_wheel2_joint, fl_wheel1_joint, fl_wheel2_joint, br_wheel1_joint, br_wheel2_joint, bl_wheel1_joint, bl_wheel2_joint]

print(motors[0].Kp)
controller = js_ctrl.JoystickController("logitech", m, d, motors)


# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running():
    step_start = time.time()

    # Call joystick controller:
    controller.control(m,d)
    # br_wheel1_joint.log_data()
    # Step the simulation forward
    mujoco.mj_step(m, d)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)


