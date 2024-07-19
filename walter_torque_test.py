import time
import mujoco
import mujoco.viewer
import numpy as np
import pygame
import lib.MotorModel as motor
import matplotlib.pyplot as plt
import matplotlib

# Initializing joystick stuff
pygame.init()
global js
js = pygame.joystick.Joystick(0)
js.init()
print(js.get_name())

def nearest_pi(current_angle):
  near_pi = np.round(current_angle/np.pi)*np.pi
  return near_pi



# Joystick control function
def control(m,d):
  js_deadzone = 0.1
  pygame.event.get()
  left_stick_lr = js.get_axis(0)
  left_stick_ud = js.get_axis(1)
  left_trigger = js.get_axis(2)
  right_stick_lr = js.get_axis(3)
  right_stick_ud = js.get_axis(4)
  right_trigger = js.get_axis(5)
  a_button = js.get_button(0)
  b_button = js.get_button(1)
  x_button = js.get_button(2)
  y_button = js.get_button(3)
  start_button = js.get_button(7)
  back_button = js.get_button(6)
  home_button = js.get_button(8)
  # hat = js.get_hat(0)

  # Reset the simulation
  if(start_button):
    mujoco.mj_resetData(m, d)


  # Control hip angles with buttons
  if(y_button):
    control.fr_hip_angle_des = np.pi/4
    control.fl_hip_angle_des = -np.pi/4
    control.br_hip_angle_des = -np.pi/4
    control.bl_hip_angle_des = np.pi/4
  elif(x_button):
    control.fr_hip_angle_des = 0
    control.fl_hip_angle_des = 0
    control.br_hip_angle_des = 0
    control.bl_hip_angle_des = 0

  # Apply hip control:
  fr_hip.pos_control(control.fr_hip_angle_des)
  fl_hip.pos_control(control.fl_hip_angle_des)
  br_hip.pos_control(control.br_hip_angle_des)
  bl_hip.pos_control(control.bl_hip_angle_des)

  # Add deadzone to joystick inputs
  if(abs(left_stick_lr) < js_deadzone):
    left_stick_lr = 0
  if(abs(left_stick_ud) < js_deadzone):
    left_stick_ud = 0
  if(abs(right_stick_lr) < js_deadzone):
    right_stick_lr = 0
  if(abs(right_stick_ud) < js_deadzone):
    right_stick_ud = 0

  # Control knee angles with right stick
  # Default knee angles:
  max_knee_vel = 0.2
  left_knee_vel = max_knee_vel*(right_stick_lr - right_stick_ud)
  right_knee_vel = max_knee_vel*(right_stick_lr + right_stick_ud)

  fr_knee_current = d.jnt('fr_knee').qpos[0]
  fl_knee_current = d.jnt('fl_knee').qpos[0]
  br_knee_current = d.jnt('br_knee').qpos[0]
  bl_knee_current = d.jnt('bl_knee').qpos[0]

  fr_knee_angle_des = fr_knee_current + right_knee_vel
  fl_knee_angle_des = fl_knee_current + left_knee_vel
  br_knee_angle_des = br_knee_current + right_knee_vel
  bl_knee_angle_des = bl_knee_current + left_knee_vel

  # Apply knee control:
  fr_knee.pos_control(fr_knee_angle_des)
  fl_knee.pos_control(fl_knee_angle_des)
  br_knee.pos_control(br_knee_angle_des)
  bl_knee.pos_control(bl_knee_angle_des)

  # Control wheel velocities with left stick
  max_wheel_vel = 1.4
  control.left_wheel_vel_des = max_wheel_vel*(left_stick_lr - left_stick_ud)
  control.right_wheel_vel_des = max_wheel_vel*(left_stick_lr + left_stick_ud)

  # Apply wheel control:
  fr_wheel1_joint.torque_control(control.right_wheel_vel_des)
  fr_wheel2_joint.torque_control(control.right_wheel_vel_des)
  fl_wheel1_joint.torque_control(control.left_wheel_vel_des)
  fl_wheel2_joint.torque_control(control.left_wheel_vel_des)
  br_wheel1_joint.torque_control(control.right_wheel_vel_des)
  br_wheel2_joint.torque_control(control.right_wheel_vel_des)
  bl_wheel1_joint.torque_control(control.left_wheel_vel_des)
  bl_wheel2_joint.torque_control(control.left_wheel_vel_des)


control.fr_hip_angle_des = 0
control.fl_hip_angle_des = 0
control.br_hip_angle_des = 0
control.bl_hip_angle_des = 0

control.left_wheel_vel_des = 0
control.right_wheel_vel_des = 0

control.bl_wheel2_angle_des = 0
control.bl_wheel1_angle_des = 0
control.br_wheel2_angle_des = 0
control.br_wheel1_angle_des = 0
control.fl_wheel2_angle_des = 0
control.fl_wheel1_angle_des = 0
control.fr_wheel2_angle_des = 0
control.fr_wheel1_angle_des = 0




# Initialize motor models

hipParams = {
  'Kp': 10,
  'Kd': 1,
  'gear_ratio': 6,
  't_stall': 2.6,
  'w_no_load': 1400*0.1047,
}

kneeParams = {
  'Kp': 50,
  'Kd': 10,
  'gear_ratio': 6*(30/19),
  't_stall': 1.7,
  'w_no_load': 5000*0.1047,
}

wheelParams = {
  'Kp': 0.1,
  'Kd': 0.05,
  'gear_ratio': 1,
  't_stall': 1.4,
  'w_no_load': 23,
}

# Load in the model and data from xml file
m = mujoco.MjModel.from_xml_path('models/walter/scene.xml')
d = mujoco.MjData(m)


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


torques = []
omegas = []

# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running():
    step_start = time.time()
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.

    control(m,d)
    mujoco.mj_step(m, d)
    fr_wheel1_joint.debug()
    torques.append(abs(fr_wheel1_joint.limited_torque[0]))
    omegas.append(abs(d.jnt('fr_wheel1_joint').qvel[0])*fr_wheel1_joint.gear_ratio)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)


# Plot the speed torque curve
w_range = np.linspace(0, wheelParams['w_no_load'], 1000)
T_line = -wheelParams['t_stall']/wheelParams['w_no_load']*w_range + wheelParams['t_stall']
matplotlib.use('QtAgg')
plt.plot(omegas, torques, 'b*')
plt.xlabel('Angular Velocity (rad/s)')
plt.ylabel('Torque (Nm)')
plt.plot(w_range, T_line, 'r--')
plt.show()