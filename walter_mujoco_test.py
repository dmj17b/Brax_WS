import time
import mujoco
import mujoco.viewer
import numpy as np
import pygame

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
  hat = js.get_hat(0)

  if(start_button):
    mujoco.mj_resetData(m, d)


  # Add deadzone to joystick inputs
  if(abs(left_stick_lr) < js_deadzone):
    left_stick_lr = 0
  if(abs(left_stick_ud) < js_deadzone):
    left_stick_ud = 0
  if(abs(right_stick_lr) < js_deadzone):
    right_stick_lr = 0
  if(abs(right_stick_ud) < js_deadzone):
    right_stick_ud = 0


  # Control signals
  '''
  fr_hip = 0
  fr_knee = 1
  fr_wheel1_joint = 2
  fr_wheel2_joint = 3
  fl_hip = 4
  fl_knee = 5
  fl_wheel1_joint = 6
  fl_wheel2_joint = 7
  br_hip = 8
  br_knee = 9
  br_wheel1_joint = 10
  br_wheel2_joint = 11
  bl_hip = 12
  bl_knee = 13
  bl_wheel1_joint = 14
  bl_wheel2_joint = 15
'''
  # Get knee angles and figure out the closest multiple of pi
  br_knee_angle = d.joint('br_knee').qpos
  bl_knee_angle = d.joint('bl_knee').qpos
  fr_knee_angle = d.joint('fr_knee').qpos
  fl_knee_angle = d.joint('fl_knee').qpos
  closest_br = nearest_pi(br_knee_angle)
  closest_bl = nearest_pi(bl_knee_angle)
  closest_fr = nearest_pi(fr_knee_angle)
  closest_fl = nearest_pi(fl_knee_angle)


  # # Hat will control hip splay if no button is pressed
  # control.hip_splay += 0.005*hat[1]
  # d.ctrl[0] = control.hip_splay
  # d.ctrl[4] = -control.hip_splay
  # d.ctrl[8] =  -control.hip_splay
  # d.ctrl[12] = control.hip_splay


  # A button will extend hips and put knees in "T" position
  if(a_button):
    d.ctrl[9] = closest_br
    d.ctrl[13] = closest_bl
    d.ctrl[0] = 3.14/2
    d.ctrl[4] = -3.14/2
    d.ctrl[8] = -3.14/2
    d.ctrl[12] = 3.14/2 
  # B button will put robot in standing start position
  elif(b_button):
    # Back knees just off from 90 degrees
    d.ctrl[9] = closest_br - np.pi/3
    d.ctrl[13] = closest_bl + np.pi/3
    # Front knees parallel with thigh
    d.ctrl[1] = closest_fr-np.pi/2
    d.ctrl[5] = closest_fl+np.pi/2
    # Hips straight
    d.ctrl[0] = -0.3
    d.ctrl[4] = 0.3
    d.ctrl[8] = 0.5
    d.ctrl[12] = -0.5
  # Otherwise, the knees will be controlled by the hat
  # else:

  # Right stick will control wheels
  left_wheels = 0.07*(left_stick_lr - left_stick_ud)
  right_wheels = 0.07*(left_stick_lr + left_stick_ud)
  d.ctrl[2] = d.ctrl[2] + right_wheels
  d.ctrl[3] = d.ctrl[3] + right_wheels
  d.ctrl[6] = d.ctrl[6] + left_wheels
  d.ctrl[7] = d.ctrl[7] + left_wheels
  d.ctrl[10] = d.ctrl[10] + right_wheels
  d.ctrl[11] = d.ctrl[11] + right_wheels
  d.ctrl[14] = d.ctrl[14] + left_wheels
  d.ctrl[15] = d.ctrl[15] + left_wheels



  # Left stick will control knees
  left_knees = 0.01*(right_stick_lr - right_stick_ud)
  right_knees = 0.01*(right_stick_lr + right_stick_ud)

  d.ctrl[1] = d.ctrl[1] + right_knees
  d.ctrl[5] = d.ctrl[5] + left_knees
  d.ctrl[9] = d.ctrl[9] + right_knees
  d.ctrl[13] = d.ctrl[13] + left_knees


control.hip_splay = 0

# Load in the model and data from xml file
m = mujoco.MjModel.from_xml_path('models/walter/scene_smalter.xml')
d = mujoco.MjData(m)
d.qpos = m.keyframe('home').qpos
d.ctrl = m.keyframe('home').qpos[8:]

# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running():
    step_start = time.time()
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    # control(m,d)
    mujoco.mj_step(m, d)
    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
