import time
import mujoco
import mujoco.viewer
import numpy as np
import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl
import lib.TestControl as test_ctrl
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

wheelParams = {
  'Kp': 10,
  'Kd': 5,
  'gear_ratio': 1,
  't_stall': 250,
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


# Initializing joystick controller object
# controller = js_ctrl.JoystickController("logitech", m, d, motors)
controller = test_ctrl.TestController(m, d, motors)

torques = []
vels = []
begin = False
# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.lookat[:] = [d.qpos[0], -2, 1]  # Set the camera's look-at point (center of view)
    viewer.cam.azimuth = 60  # Set the azimuth (horizontal angle)
    viewer.cam.elevation = -20  # Set the elevation (vertical angle)
    viewer.cam.distance = 5 # Set the distance from the camera to the object

    start = time.time()
    while viewer.is_running():


        step_start = time.time()
        viewer.cam.lookat[:] = [d.qpos[0], -3, 1]  # Set the camera's look-at point (center of view)

        # Step the simulation forward
        mujoco.mj_step(m, d)
        # Call joystick controller:
        if(keyboard.is_pressed(' ')):
            begin = True
        if(keyboard.is_pressed('esc')):
            break

        if(begin):
            controller.control(m,d)
            torques.append(d.qfrc_actuator[:])
            vels.append(d.qvel[:])
        else:
            controller.start_pos()

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


# Calculate COT:
torques = np.array(torques)
energy_elec = np.sum(np.trapezoid(torques ** 2, dx=m.opt.timestep, axis=0), axis=-1)
energy_mech = np.sum(np.trapezoid(np.maximum(torques * vels,0), dx=m.opt.timestep, axis=0), axis=-1)
print(m.body_mass)
print(sum(m.body_mass[0:19]))
print(d.qpos[0])
cot_elec = energy_elec / (sum(m.body_mass[0:19]) * -m.opt.gravity[2] * d.qpos[0])
cot_mech = energy_mech / (sum(m.body_mass[0:19]) * -m.opt.gravity[2] * d.qpos[0])
print("Electrical COT: ", cot_elec)
print("Mechanical COT: ", cot_mech)


