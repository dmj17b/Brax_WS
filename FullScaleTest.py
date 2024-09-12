import time
import mujoco
import mujoco.viewer
import numpy as np
import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl
import lib.TestControl as test_ctrl
import keyboard
import matplotlib.pyplot as plt

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
controller = test_ctrl.TestController(m, d, motors)


# Initialize logging variables
torques = []
vels = []
begin = False
power_mech = []
power_elec = []
run_time = 0
time_vec = []

joint_idx = [6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22]
# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.lookat[:] = [d.qpos[0], -2, 1]  # Set the camera's look-at point (center of view)
    viewer.cam.azimuth = 60  # Set the azimuth (horizontal angle)
    viewer.cam.elevation = -20  # Set the elevation (vertical angle)
    viewer.cam.distance = 5 # Set the distance from the camera to the object

    start = time.time()
    while viewer.is_running():


        step_start = time.time()
        viewer.cam.lookat[:] = [d.qpos[0], -1.5, 1.5]  # Set the camera's look-at point (center of view)

        # Step the simulation forward
        mujoco.mj_step(m, d)
        # Call joystick controller:
        if(keyboard.is_pressed(' ')):
            begin = True
        if(keyboard.is_pressed('esc')):
            break

        if(begin):
            # Call controller: 
            controller.control(m,d)

            # Log time :
            run_time += m.opt.timestep
            time_vec.append(run_time)

            # Log torques and velocities:
            torques.append(d.qfrc_actuator[joint_idx])
            vels.append(d.qvel[joint_idx])
            body_vel = d.qvel[0:2]

            # Calculate and log power:
            trq = d.qfrc_actuator[joint_idx]
            vel = d.qvel[joint_idx]
            power_elec.append(np.dot(trq,trq))
            power_mech.append(np.maximum(np.dot(trq,vel),0))


        else:
            controller.start_pos()

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        if(d.qpos[0]>5.4):
            break



# Calculate COT:
torques = np.array(torques)
energy_elec = np.sum(np.trapezoid(torques ** 2, dx=m.opt.timestep, axis=0), axis=-1)
energy_mech = np.sum(np.trapezoid(np.maximum(torques * vels,0), dx=m.opt.timestep, axis=0), axis=-1)
dist = np.sqrt(d.qpos[0]**2 + d.qpos[2]**2)
print("Distance travelled: ", dist)
cot_elec = energy_elec / (sum(m.body_mass[0:19]) * -m.opt.gravity[2] * dist)
cot_mech = energy_mech / (sum(m.body_mass[0:19]) * -m.opt.gravity[2] * dist)
print("Electrical COT: ", cot_elec)
print("Mechanical COT: ", cot_mech)

# Calculate average velocity:
v_avg = dist/run_time

# Plot power:
power_mech = np.array(power_mech)
power_elec = np.array(power_elec)
time_vec = np.array(time_vec)

plt.figure()
plt.plot(time_vec, power_mech, label='Mechanical Power')
plt.plot(time_vec, power_elec, label='Electrical Power')
plt.xlabel('Time (s)')
plt.ylabel('Power (W)')
plt.legend()
plt.show()

# Combine data into a csv file:
folder = 'TestData/'
filename = 'RandomTest1'
data = np.vstack((time_vec, power_mech, power_elec))
data = np.asarray(data).T
np.savetxt(folder+filename+'.csv', data, delimiter=',')

results = np.array([["COT_ELEC", cot_elec], ["COT_MECH", cot_mech], ["Vel", v_avg]])
with open(folder+filename+'.txt', 'w') as f:
    for item in results:
        f.write("%s\n" % item)
    f.close()