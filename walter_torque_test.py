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
  # hat = js.get_hat(0)

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
  d.ctrl[0] = 10
 
def pos_control(m: mujoco.MjModel, d: mujoco.MjData, target_pos: float, joint_name: str):
  print(d.jnt)



# Load in the model and data from xml file
m = mujoco.MjModel.from_xml_path('models/walter/scene.xml')
d = mujoco.MjData(m)
# d.qpos = m.keyframe('home').qpos
# d.ctrl = m.keyframe('home').qpos[8:]

# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running():
    step_start = time.time()
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    control(m,d)
    pos_control(m,d,'DJ',1.0)
    mujoco.mj_step(m, d)
    print(d.joint('fr_hip').qvel)
    


    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
