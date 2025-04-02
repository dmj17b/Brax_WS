import time
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
import matplotlib.pyplot as plt
import zmq
import lib.Sender as sender


# Call AutoSim to generate the new robot spec:
model_config_path = os.getcwd() + '/model_configs/Test/model_config.yaml'

# Load frameless motor options:
# motor_config_path = os.getcwd() + '/motor_configs/frameless.yaml'

#
motor_config_path = os.getcwd() + '/motor_configs/myactuator.yaml'


motor_config = yaml.safe_load(Path(motor_config_path).read_text())

# Generate the new robot spec:
walter = AutoSim.GenerateModel(model_config_path=model_config_path, motor_config_path=motor_config_path)

#Add payload to walter body
walter.add_payload(mass = 32, body_loc = [0,0,0.2], size = [0.2, 0.1, 0.1])

# Generate the scene around the robot (groundplane and sky)
walter.gen_scene()

# Add some obstacles:
walter.add_stairs(rise=0.2,run=0.3,num_steps=15)
walter.add_log(d=0.4,length = 2)
walter.add_incline(angle_deg=40, pos = [3, 5, 0], width = 1.5, length = 4 )
walter.add_box(pos = [5.5, 5, 2.5], size = [1, 2, 0.1])
walter.add_incline(angle_deg=-40, pos = [8, 5, 0], width = 1.5, length = 4 )

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


# Select motors to plot (you can modify this list as needed)
global motors_to_plot
motors_to_plot = [br_hip, br_knee, br_wheel1_joint]

logger = sender.DataSender()



# Main simulation loop:
with mujoco.viewer.launch_passive(m,d,show_left_ui=False,show_right_ui=False) as viewer:
    start = time.time()
    logger.last_log_time = time.time() - start

    while viewer.is_running():
        step_start = time.time()
        sim_time = time.time() - start

        # Step the simulation forward
        mujoco.mj_step(m, d)

        # Call joystick controller:
        m,d = controller.control(m,d)

        vel = d.qvel[0:2]
        abs_vel = np.linalg.norm(vel)
        vel_kph = abs_vel * 3.6
        print(f"Velocity: {vel_kph:.2f} km/h")

        # Save desired motor data:
        motors_to_plot = motor.log_motor_data_output(motors_to_plot)

        # Put together data struct for plotter:
        data = {
            'time': time.time()-start,
            # 'motor1_data': abs_vel
            'motor1_torque': motors_to_plot[0].torques[-1],
            'motor1_speed': motors_to_plot[0].omegas[-1],
            'motor2_torque': motors_to_plot[1].torques[-1],
            'motor2_speed': motors_to_plot[1].omegas[-1],
            'motor3_torque': motors_to_plot[2].torques[-1],
            'motor3_speed': motors_to_plot[2].omegas[-1],
        }


        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Send data to the plotter
        logger.send_data(sim_time, data)
        

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# motor.plot_motor_data_output(motors_to_plot)
# br_wheel1_joint.plot_data_output_rpms()
