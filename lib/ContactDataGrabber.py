import numpy as np
import pandas as pd
import mujoco

def get_motor_targets(controller):
    #TODO: Fix wheel target velocities. Make sure they match between joystick and random controllers
    target_positions = []
    target_positions.append(controller.fr_hip_des_pos)
    target_positions.append(controller.fl_hip_des_pos)
    target_positions.append(controller.br_hip_des_pos)
    target_positions.append(controller.bl_hip_des_pos)
    target_positions.append((controller.fr_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
    target_positions.append((controller.fl_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
    target_positions.append((controller.br_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
    target_positions.append((controller.bl_knee_des_pos + np.pi) % (2*np.pi) - np.pi)
    target_positions.append(controller.wheel1_des_vel)
    target_positions.append(controller.wheel2_des_vel)
    target_positions.append(controller.wheel3_des_vel)
    target_positions.append(controller.wheel4_des_vel)
    target_positions.append(controller.wheel5_des_vel)
    target_positions.append(controller.wheel6_des_vel)
    target_positions.append(controller.wheel7_des_vel)
    target_positions.append(controller.wheel8_des_vel)
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

def get_projected_gravity(m, d, sensor_name):
    beta = 0.01
    acc_data = d.sensor(f"{sensor_name}_acc").data
    gyro_data = d.sensor(f"{sensor_name}_gyro").data


    # If first call, initialize g_est
    if not hasattr(get_projected_gravity, "g_est"):
        get_projected_gravity.g_est = -acc_data / np.linalg.norm(acc_data)



    g_pred = get_projected_gravity.g_est + m.opt.timestep*np.cross(gyro_data, get_projected_gravity.g_est)
    g_pred /= np.linalg.norm(g_pred)

    a_dir = -acc_data / np.linalg.norm(acc_data)

    g_est = (1-beta) * g_pred + beta * a_dir
    g_est /= np.linalg.norm(g_est)

    return g_est


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


def extract_observation(actual_positions, target_positions, motor_torques, head_projected_grav, torso_projected_grav):
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
    
    # Add projected gravity components
    # obs['head_grav_x'] = head_projected_grav[0]
    # obs['head_grav_y'] = head_projected_grav[1]
    # obs['head_grav_z'] = head_projected_grav[2]
    # obs['torso_grav_x'] = torso_projected_grav[0]
    # obs['torso_grav_y'] = torso_projected_grav[1]
    # obs['torso_grav_z'] = torso_projected_grav[2]

    
    return obs

def extract_observation_with_noise(actual_positions, target_positions, motor_torques, head_projected_grav, torso_projected_grav):
    """
    Extract observation features (without contacts) as a dictionary.
    This will be used for both current and historical observations.
    """

    state_noise = 0.001  # Small noise to add to state features
    error_noise = 0.001  # Small noise to add to error features
    torque_noise = 0.01  # Small noise to add to torque features
    gravity_noise = 0.001  # Small noise to add to gravity features

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

    # # Add projected gravity components
    # obs['head_grav_x'] = head_projected_grav[0] + np.random.uniform(-gravity_noise, gravity_noise)
    # obs['head_grav_y'] = head_projected_grav[1] + np.random.uniform(-gravity_noise, gravity_noise)
    # obs['head_grav_z'] = head_projected_grav[2] + np.random.uniform(-gravity_noise, gravity_noise)
    # obs['torso_grav_x'] = torso_projected_grav[0] + np.random.uniform(-gravity_noise, gravity_noise)
    # obs['torso_grav_y'] = torso_projected_grav[1] + np.random.uniform(-gravity_noise, gravity_noise)
    # obs['torso_grav_z'] = torso_projected_grav[2] + np.random.uniform(-gravity_noise, gravity_noise)
    return obs


def extract_obs_array(actual_positions, target_positions, motor_torques, torso_projected_grav, head_projected_grav):
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
    
    # obs.extend(torso_projected_grav)
    # obs.extend(head_projected_grav)
    
    return np.array(obs)