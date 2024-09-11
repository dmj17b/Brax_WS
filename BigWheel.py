import time
import mujoco
import mujoco.viewer
import numpy as np
import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl
import keyboard 

# Load in the model and data from xml file
m = mujoco.MjModel.from_xml_path('models/walter/scene.xml')
d = mujoco.MjData(m)

# Initializing motor model parameters:
hip_kv = 150
hip_voltage = 4*12  # 12 cell battery pack
knee_kv = 230
knee_voltage = 4*12 # 12 cell battery pack

hipParams = {
  'Kp': 800,
  'Kd': 80,
  'gear_ratio': 1,
  't_stall': 150,
  'w_no_load': 230*0.1047,
}

kneeParams = {
  'Kp': 800,
  'Kd': 80,
  'gear_ratio': 1,
  't_stall': 150,
  'w_no_load': 230*0.1047,
}


# Initializing motor models
fr_hip = motor.MotorModel(m, d, 'fr_hip', hipParams, 0)
fl_hip = motor.MotorModel(m, d,'fl_hip', hipParams, 2)
br_hip = motor.MotorModel(m, d,'br_hip', hipParams, 4)
bl_hip = motor.MotorModel(m, d,'bl_hip', hipParams, 6)

fr_knee = motor.MotorModel(m, d, 'fr_knee', kneeParams, 1)
fl_knee = motor.MotorModel(m, d, 'fl_knee', kneeParams, 3)
br_knee = motor.MotorModel(m, d, 'br_knee', kneeParams, 5)
bl_knee = motor.MotorModel(m, d, 'bl_knee', kneeParams, 7)


motors = [fr_hip, fl_hip, br_hip, bl_hip, 
          fr_knee, fl_knee, br_knee, bl_knee]



# Initializing joystick controller object
# controller = js_ctrl.JoystickController("logitech", m, d, motors)
controller = 

torques = []
# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.lookat[:] = [d.qpos[0], -2, 1]  # Set the camera's look-at point (center of view)
    viewer.cam.azimuth = 60  # Set the azimuth (horizontal angle)
    viewer.cam.elevation = -20  # Set the elevation (vertical angle)
    viewer.cam.distance = 5 # Set the distance from the camera to the object  start = time.time()
    while viewer.is_running():
      step_start = time.time()

      # Step the simulation forward
      mujoco.mj_step(m, d)

      # Call joystick controller:

      controller.control(m,d)
      br_knee.log_data()
      br_hip.log_data()

      torques.append(d.qfrc_actuator[:])

      # Pick up changes to the physics state, apply perturbations, update options from GUI.
      viewer.sync()

      

      # Rudimentary time keeping, will drift relative to wall clock.
      time_until_next_step = m.opt.timestep - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)

      if(d.qpos[0]>10):
        break


# Calculate COT:
torques = np.array(torques)
energy = np.sum(np.trapezoid(torques ** 2, dx=m.opt.timestep, axis=0), axis=-1)
print(sum(m.body_mass[0:11]))
print(d.qpos[0])
cot = energy / (sum(m.body_mass[0:11]) * -m.opt.gravity[2] * d.qpos[0])
print(cot)

