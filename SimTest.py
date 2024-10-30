import time
import mujoco
import mujoco.viewer
import numpy as np
import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl

# Load in the model and data from xml file
spec = mujoco.MjSpec.from_file('models/walter/scene.xml')



# Here is where you can adjust SOME of the model parameters for the robot
modelParams = {
  'head_mass': 1,
  'body_mass': 1,
  'thigh_mass': 100,
  'shin_mass': 0.5,
  'wheel_mass': 0.5,
}


# Here is where you can adjust all of the actuator parameters
# Kp and Kd are the proportional and derivative gains of the motor controller (position control for hip and knee, velocity control for wheels)
# For the wheels, only Kp is used to track a desired velocity
# gear_ratio is the gear ratio of the motor, IT DOES NOT PROPERLY AFFECT REFLECTED INERTIA YET
# t_stall is the stall torque of the motor in Nm
# w_no_load is the no load speed of the motor in rad/s


hipParams = {
  'Kp': 600,
  'Kd': 80,
  'gear_ratio': 1,
  't_stall': 120,
  'w_no_load': 230*0.1047,
  'rotor_inertia': 0.0001,
}

kneeParams = {
  'Kp': 600,
  'Kd': 30,
  'gear_ratio': 1,
  't_stall': 100,
  'w_no_load': 230*0.1047,
  'rotor_inertia': 0.0001,
}

wheelParams = {
  'Kp': 0.5,
  'Kd': 0.3,
  'gear_ratio': 1,
  't_stall': 25,
  'w_no_load': 230*0.1047,
  'rotor_inertia': 10000,
}


# Compile model based on new parameters:
spec.geoms[1].mass = modelParams['body_mass']
spec.geoms[2].mass = modelParams['thigh_mass']
spec.geoms[3].mass = modelParams['shin_mass']
spec.geoms[4].mass = modelParams['wheel_mass']
spec.geoms[5].mass = modelParams['wheel_mass']
spec.geoms[6].mass = modelParams['thigh_mass']
spec.geoms[7].mass = modelParams['shin_mass']
spec.geoms[8].mass = modelParams['wheel_mass']
spec.geoms[9].mass = modelParams['wheel_mass']
spec.geoms[10].mass = modelParams['head_mass']
spec.geoms[11].mass = modelParams['thigh_mass']
spec.geoms[12].mass = modelParams['shin_mass']
spec.geoms[13].mass = modelParams['wheel_mass']
spec.geoms[14].mass = modelParams['wheel_mass']
spec.geoms[15].mass = modelParams['thigh_mass']
spec.geoms[16].mass = modelParams['shin_mass']
spec.geoms[17].mass = modelParams['wheel_mass']
spec.geoms[18].mass = modelParams['wheel_mass']



# Adjusting armatures:
spec.find_default('hip').joint.armature = hipParams['rotor_inertia']*hipParams['gear_ratio']**2
spec.find_default('knee').joint.armature = kneeParams['rotor_inertia']*kneeParams['gear_ratio']**2
spec.find_default('wheel_joint').joint.armature = wheelParams['rotor_inertia']*wheelParams['gear_ratio']**2

m = spec.compile()
d = mujoco.MjData(m)




# Initializing motor models (ignore this part for now)
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



# Initializing joystick controller object (ignore this part unless controller axes are changed)
controller = js_ctrl.JoystickController("logitech2", m, d, motors)


# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running():
    step_start = time.time()

    # Step the simulation forward
    mujoco.mj_step(m, d)

    # Call joystick controller:
    controller.control(m,d)


    # Log motor data to plot later:

    # br_wheel1_joint.log_data()
    # br_knee.log_data()
    # br_hip.log_data()


    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()


    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)


# Plot desired speed-torque curves

# br_wheel1_joint.plot_speed_torque_curve()
# br_knee.plot_speed_torque_curve()
# br_hip.plot_speed_torque_curve()