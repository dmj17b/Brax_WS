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
walter.gen_terrain()


# Compile the model:
m = walter.spec.compile()
d = mujoco.MjData(m)

m.opt.timestep = 0.001
m.geom_margin = -0.005

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

contact_network = tf.keras.models.load_model('contact_predictor/best_contact_predictor_model.keras')
contact_network.compile()  # Ensure model is compiled for faster inference
std_color = np.array([177/255, 166/255, 136/255, 1])

step_count = 0
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

        head_projected_grav = 0
        torso_projected_grav = 0


        # Extract current observation (without contacts)
        current_obs = cdg.extract_obs_array(actual_positions,
                                            target_positions,
                                            motor_torques, 
                                            head_projected_grav, 
                                            torso_projected_grav)


        true_contacts = cdg.get_sim_wheel_collisions(m,d)
        filtered_true_contacts = cdg.filter_sim_wheel_collisions(true_contacts)

        if step_count % 10 == 0:
            # Call contact data grabber to get true wheel contacts


            # Call contact predictor to predict wheel contacts
            input_obs = np.expand_dims(current_obs, axis=0).astype(np.float32)  # Add batch dimension: (1, 48)
            predicted_contacts = contact_network(input_obs, training=False).numpy()  # Direct call instead of predict()
            predicted_contacts = (predicted_contacts[0] > 0.5).astype(int)  # Binarize predictions at 0.5 threshold

            # Change color of wheels based on predicted contacts
            #Back left wheels:
            m.geom_rgba[4] = [1, 0, 0, 1] if predicted_contacts[0] == 1 else std_color  
            m.geom_rgba[5] = [1, 0, 0, 1] if predicted_contacts[1] == 1 else std_color 

            #Back right wheels:
            m.geom_rgba[8] = [1, 0, 0, 1] if predicted_contacts[2] == 1 else std_color
            m.geom_rgba[9] = [1, 0, 0, 1] if predicted_contacts[3] == 1 else std_color

            #Front left wheels:
            m.geom_rgba[13] = [1, 0, 0, 1] if predicted_contacts[4] == 1 else std_color
            m.geom_rgba[14] = [1, 0, 0, 1] if predicted_contacts[5] == 1 else std_color

            #Front right wheels:
            m.geom_rgba[17] = [1, 0, 0, 1] if predicted_contacts[6] == 1 else std_color
            m.geom_rgba[18] = [1, 0, 0, 1] if predicted_contacts[7] == 1 else std_color

            # Calcualate running accuracy:
            step_accuracy = np.mean(predicted_contacts == filtered_true_contacts)
            # Accumulate accuracy and count prediction steps
            if step_count == 0:
                step_accuracy_sum = 0
            step_accuracy_sum += step_accuracy
            running_accuracy = step_accuracy_sum / (step_count/10)
            print(f"Predicted Contacts: {predicted_contacts}\nSimulated Contacts: {filtered_true_contacts}\n")
            print(f"Total Simulation Contact Prediction Accuracy: {running_accuracy*100:.2f}%\n")
        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
        step_count += 1
        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = (m.opt.timestep - (time.time() - step_start))
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

            