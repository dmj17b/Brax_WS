import numpy as np
import pandas as pd
import mujoco

def get_motor_targets(controller):
    target_positions = []
    target_positions.append(controller.fr_hip_des_pos)
    target_positions.append(controller.fl_hip_des_pos)
    target_positions.append(controller.br_hip_des_pos)
    target_positions.append(controller.bl_hip_des_pos)
    target_positions.append((controller.fr_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
    target_positions.append((controller.fl_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
    target_positions.append((controller.br_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
    target_positions.append((controller.bl_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
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
        if "wheel_joint" in motor.motor_name:
            q = motor.d.jnt(motor.motor_name).qvel
        if "shin_joint" in motor.motor_name:
            q = (motor.d.jnt(motor.motor_name).qpos+np.pi) % (2*np.pi) - np.pi
        if "thigh_joint" in motor.motor_name:
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
def get_body_orientation(m, d, body_name):
    body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
    quat = d.xquat[body_id]
    return quat

def get_wheel_sensor_data(m, d):
    """
    Get wheel distance sensor readings.
    Returns array of distances from wheels to ground.
    """
    sensor_data = []
    sensor_names = [
        'bl_front_wheel_dist', 'bl_rear_wheel_dist',
        'br_front_wheel_dist', 'br_rear_wheel_dist',
        'fl_front_wheel_dist', 'fl_rear_wheel_dist',
        'fr_front_wheel_dist', 'fr_rear_wheel_dist'
    ]
    
    for sensor_name in sensor_names:
        sensor_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id >= 0:
            # Get sensor address in sensordata array
            sensor_adr = m.sensor_adr[sensor_id]
            sensor_data.append(d.sensordata[sensor_adr])
        else:
            sensor_data.append(np.nan)  # Sensor not found
    
    return np.array(sensor_data)

def infer_contacts(wheel_distances, threshold=0.005):
    """
    Infer wheel contacts based on distance sensor readings.
    
    Args:
        wheel_distances: numpy array of wheel distance sensor readings
        threshold: distance threshold below which contact is inferred
    
    Returns:
        numpy array with 1 if wheel is in contact, 0 otherwise
    """
    contacts = (abs(wheel_distances) < threshold).astype(int)
    return contacts

def extract_observation(actual_positions, target_positions, motor_torques, torso_quat, head_quat):
    """
    Extract observation features (without contacts) as a dictionary.
    This will be used for both current and historical observations.
    """
    obs = {}
    
    # Add actual positions
    for i, pos in enumerate(actual_positions):
        obs[f'actual_{i}'] = pos
    
    # Add errors
    error_vec = np.array(target_positions) - np.array(actual_positions)
    for i, error in enumerate(error_vec):
        obs[f'error_{i}'] = error*100
    
    # Add motor torques
    for i, torque in enumerate(motor_torques):
        obs[f'torque_{i}'] = torque
    

    # obs['head_quat_z'] = head_quat[3]
    
    return obs

def extract_observation_with_noise(actual_positions, target_positions, motor_torques, torso_quat, head_quat):
    """
    Extract observation features (without contacts) as a dictionary.
    This will be used for both current and historical observations.
    """

    state_noise = 0.001  # Small noise to add to state features
    error_noise = 0.001  # Small noise to add to error features
    torque_noise = 0.01  # Small noise to add to torque features

    obs = {}
    
    # Add actual positions
    for i, pos in enumerate(actual_positions):
        obs[f'actual_{i}'] = pos + np.random.uniform(-state_noise, state_noise)
    
    # Add errors
    error_vec = np.array(target_positions) - np.array(actual_positions)
    for i, error in enumerate(error_vec):
        obs[f'error_{i}'] = error*100 + np.random.uniform(-error_noise, error_noise)
    
    # Add motor torques
    for i, torque in enumerate(motor_torques):
        obs[f'torque_{i}'] = torque + np.random.uniform(-torque_noise, torque_noise)

    # obs['head_quat_z'] = head_quat[3]
    
    return obs


def extract_obs_array(actual_positions, target_positions, motor_torques, torso_quat, head_quat):
    """
    Extract observation features (without contacts) as a numpy array.
    This will be used for both current and historical observations.
    """
    obs = []
    
    # Add actual positions
    obs.extend(actual_positions)
    
    # Add errors
    error_vec = np.array(target_positions) - np.array(actual_positions)
    obs.extend(error_vec * 100)
    
    # Add motor torques
    obs.extend(motor_torques)
    
    # obs.append(head_quat[3])  # head_quat_z
    
    return np.array(obs)