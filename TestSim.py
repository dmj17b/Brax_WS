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
import tensorflow as tf
import keras


# Call AutoSim to generate the new robot spec:
model_config_path = 'model_configs/2_7_Scale/model_config.yaml'
motor_config_path = 'motor_configs/myactuator.yaml'

# Load motor params for later access
motor_config = yaml.safe_load(Path(motor_config_path).read_text())

# Generate the new robot spec:
walter = AutoSim.GenerateModel(model_config_path=model_config_path, motor_config_path=motor_config_path)

#Add payload to walter body
# walter.add_payload(mass = 32, body_loc = [0,0,0.2], size = [0.2, 0.1, 0.1])

# Generate the scene around the robot (groundplane and sky)
walter.gen_scene()

# Add some obstacles:

walter.add_stairs(rise=0.2,run=0.3,num_steps=15)
walter.add_log(d=0.4,length = 2)
walter.add_incline(angle_deg=40, pos = [3, 5, 0], width = 1.5, length = 4 )
walter.add_box(pos = [5.5, 5, 2.5], size = [1, 2, 0.1])
walter.add_incline(angle_deg=-40, pos = [8, 5, 0], width = 1.5, length = 4 )
walter.add_box(pos = [-2, -2, 0.05], size = [1, 1, 0.1], name = 'box2')
# Compile the model:
m = walter.spec.compile()
d = mujoco.MjData(m)

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
controller = js_ctrl.JoystickController("logitech", m, d, motors)

def get_wheel_contacts(m, d):
    """
    Returns a binary array indicating which wheels are in contact with any geometry.
    
    Returns:
        numpy array with 1 if wheel is in contact, 0 otherwise
    """
    # Find all wheel geometries
    wheel_geom_ids = []
    for i in range(m.ngeom):
        geom_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
        if geom_name and 'wheel' in geom_name.lower():
            wheel_geom_ids.append(i)
    
    # Initialize contact array
    wheel_contacts = np.zeros(len(wheel_geom_ids), dtype=int)
    
    # Check all active contacts
    for i in range(d.ncon):
        contact = d.contact[i]
        geom1 = contact.geom1
        geom2 = contact.geom2
        
        # Check if either geometry in the contact pair is a wheel
        for idx, wheel_id in enumerate(wheel_geom_ids):
            if geom1 == wheel_id or geom2 == wheel_id:
                wheel_contacts[idx] = 1
    
    return wheel_contacts

# Get desired positions and velocities from joystick controller
def get_motor_targets(controller):
    target_positions = []
    target_positions.append(controller.fr_hip_des_pos)
    target_positions.append(controller.fl_hip_des_pos)
    target_positions.append(controller.br_hip_des_pos)
    target_positions.append(controller.bl_hip_des_pos)
    target_positions.append(controller.fr_knee_des_pos)
    target_positions.append(controller.fl_knee_des_pos)
    target_positions.append(controller.br_knee_des_pos)
    target_positions.append(controller.bl_knee_des_pos)
    target_positions.append(controller.right_wheel_vel_des)
    target_positions.append(controller.right_wheel_vel_des)
    target_positions.append(controller.left_wheel_vel_des)
    target_positions.append(controller.left_wheel_vel_des)
    target_positions.append(controller.right_wheel_vel_des)
    target_positions.append(controller.right_wheel_vel_des)
    target_positions.append(controller.left_wheel_vel_des)
    target_positions.append(controller.left_wheel_vel_des)
    return target_positions

# Get actual motor positions/velocities from motor models
def get_motor_positions(motors):
    actual_positions = []
    for motor in motors:
        if "wheel" in motor.motor_name:
            q = motor.d.jnt(motor.motor_name).qvel
        else:
            q = motor.d.jnt(motor.motor_name).qpos
        actual_positions.append(float(q[0]))
    return actual_positions

# Get motor torques from motor models
def get_motor_torques(motors):
    torques = []
    for motor in motors:
        torques.append(float(motor.limited_torque))
    return torques

# Get body orientation (quaternion) for a given body name
def get_body_orientation(d, body_name):
    body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    quat = d.xquat[body_id]
    return quat


# Load the trained contact predictor model
prediction_model = tf.keras.models.load_model('contact_predictor_model.keras')

def predict_contacts(model, input_data):
    """
    Predict wheel contacts using the trained model.
    
    Args:
        model: Trained TensorFlow/Keras model
        input_data: Numpy array of data inputs
        
    Returns:
        Numpy array of predicted contacts (num_samples, num_wheels)
    """
    predictions = model.predict(input_data)
    # Convert probabilities to binary contacts (0 or 1) using a threshold of 0.5
    binary_contacts = (predictions >= 0.5).astype(int)
    return binary_contacts

data_log = []
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
        wheel_contacts = get_wheel_contacts(m, d)
        actual_positions = get_motor_positions(motors)
        target_positions = get_motor_targets(controller)
        
        # Add motor torques
        motor_torques = get_motor_torques(motors)


        # Get torso orientation
        torso_quat = get_body_orientation(d, 'torso')


        # Get head orientation
        head_quat = get_body_orientation(d, 'head')

        # Predict wheel contacts:
        input_features = np.concatenate([
            np.array(actual_positions), 
            np.array(motor_torques), 
            torso_quat, 
            head_quat
        ])
        predicted_contacts = predict_contacts(prediction_model, input_features.reshape(1, -1))

        print("Predicted Contacts:", predicted_contacts, "Actual Contacts:", wheel_contacts)




        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
        

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

            
