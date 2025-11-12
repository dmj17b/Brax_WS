import time
from tkinter.font import names
import mujoco
import mujoco.viewer
import numpy as np
import yaml
from pathlib import Path
import os
import sys
print(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir,)))
import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl
import AutoSim
import pandas as pd
import lib.ContactDataGrabber as cdg
import tensorflow as tf

# Call AutoSim to generate the new robot spec:
model_config_path = 'model_configs/2_7_Scale/model_config.yaml'
motor_config_path = 'model_configs/2_7_Scale/motor_config.yaml'

# Load motor params for later access
motor_config = yaml.safe_load(Path(motor_config_path).read_text())

# Generate the new robot spec:
walter = AutoSim.GenerateModel(model_config_path=model_config_path, motor_config_path=motor_config_path)

# Generate the scene around the robot (groundplane and sky)
walter.gen_scene()

#Add wheel contact sensors
walter.add_wheel_sensors()


# Compile the model:
m = walter.spec.compile()
d = mujoco.MjData(m)

m.opt.timestep = 0.01

step_dt = 0.02

# Initializing motor models (ignore this part)
fr_hip = motor.MotorModel(m, d, 'head_right_thigh_joint', motor_config['hip_params'], 12)
fl_hip = motor.MotorModel(m, d,'head_left_thigh_joint',  motor_config['hip_params'], 8)
br_hip = motor.MotorModel(m, d,'torso_right_thigh_joint',  motor_config['hip_params'], 4)
bl_hip = motor.MotorModel(m, d,'torso_left_thigh_joint',  motor_config['hip_params'], 0)

fr_knee = motor.MotorModel(m, d, 'head_right_thigh_shin_joint', motor_config['knee_params'], 13)
fl_knee = motor.MotorModel(m, d, 'head_left_thigh_shin_joint', motor_config['knee_params'], 9)
br_knee = motor.MotorModel(m, d, 'torso_right_thigh_shin_joint', motor_config['knee_params'], 5)
bl_knee = motor.MotorModel(m, d, 'torso_left_thigh_shin_joint', motor_config['knee_params'], 1)

fr_wheel1_joint = motor.MotorModel(m, d, 'head_right_shin_front_wheel_joint', motor_config['wheel_params'], 14)
fr_wheel2_joint = motor.MotorModel(m, d, 'head_right_shin_rear_wheel_joint', motor_config['wheel_params'], 15)
fl_wheel1_joint = motor.MotorModel(m, d, 'head_left_shin_front_wheel_joint', motor_config['wheel_params'], 10)
fl_wheel2_joint = motor.MotorModel(m, d, 'head_left_shin_rear_wheel_joint', motor_config['wheel_params'], 11)
br_wheel1_joint = motor.MotorModel(m, d, 'torso_right_shin_front_wheel_joint', motor_config['wheel_params'], 6)
br_wheel2_joint = motor.MotorModel(m, d, 'torso_right_shin_rear_wheel_joint', motor_config['wheel_params'], 7)
bl_wheel1_joint = motor.MotorModel(m, d, 'torso_left_shin_front_wheel_joint', motor_config['wheel_params'], 2)
bl_wheel2_joint = motor.MotorModel(m, d, 'torso_left_shin_rear_wheel_joint', motor_config['wheel_params'], 3)

motors = [fr_hip, fl_hip, br_hip, bl_hip, 
          fr_knee, fl_knee, br_knee, bl_knee, 
          fr_wheel1_joint, fr_wheel2_joint, fl_wheel1_joint, fl_wheel2_joint, br_wheel1_joint, br_wheel2_joint, bl_wheel1_joint, bl_wheel2_joint]


# Initialize joystick controller
controller = js_ctrl.JoystickController("logitech2", m, d, motors)


data_log = []
previous_obs_history = []
num_history_steps = 5  # Number of previous steps to store
step_count = 0

# Load the trained contact predictor model
contact_network = tf.keras.models.load_model('contact_predictor/best_contact_predictor_model.keras')
contact_network.compile()  # Ensure model is compiled for faster inference

# Main simulation loop:
with mujoco.viewer.launch_passive(m,d,show_left_ui=False,show_right_ui=False) as viewer:
    start = time.time()
    while viewer.is_running():
        step_start = time.time()

        # Step the simulation forward
        mujoco.mj_step(m, d)
        # Call joystick controller:
        controller.control(m,d)

        # Get info for contact predictor
        actual_positions = cdg.get_motor_positions(motors)
        target_positions = cdg.get_motor_targets(controller)
        motor_torques = cdg.get_motor_torques(motors)
        torso_quat = cdg.get_body_orientation(m, d, 'torso')
        head_quat = cdg.get_body_orientation(m, d, 'head')

        # Extract current observation (without contacts)
        current_obs = cdg.extract_obs_array(actual_positions, target_positions, motor_torques, torso_quat, head_quat)

        # Call contact predictor to predict wheel contacts
        input_obs = np.expand_dims(current_obs, axis=0).astype(np.float32)  # Add batch dimension: (1, 48)
        predicted_contacts = contact_network(input_obs, training=False).numpy()  # Direct call instead of predict()
        predicted_contacts = (predicted_contacts[0] > 0.5).astype(int)  # Binarize predictions at 0.5 threshold

        if step_count % 10 == 0:
            print(f"Predicted Contacts: {predicted_contacts}\nSimulated Contacts: {cdg.infer_contacts(cdg.get_wheel_sensor_data(m, d) )}\n")
        
        

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
        step_count += 1

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = (m.opt.timestep - (time.time() - step_start))
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

            