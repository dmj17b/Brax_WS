import time
import mujoco
import mujoco.viewer
import numpy as np
import yaml
from pathlib import Path
import os
import sys
import threading
import queue
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Append parent directories to system path
print(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir,)))

import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl
import AutoSim

# Global data queue for motor data
motor_data_queue = queue.Queue()
stop_event = threading.Event()

# Data collection thread function
def collect_motor_data(motors_to_log):
    """
    Thread function to collect motor data
    
    :param motors_to_log: List of motor objects to log data from
    """
    while not stop_event.is_set():
        # Collect data for each motor
        motor_data = {}
        for motor_obj in motors_to_log:
            # Assuming the motor has methods to get current RPM and other relevant data
            motor_data[motor_obj.joint_name] = {
                'output_rpm': motor_obj.get_output_rpm(),  # You might need to implement this method
                'current': motor_obj.get_current(),  # You might need to implement this method
                # Add any other data you want to collect
            }
        
        # Put data in queue
        motor_data_queue.put(motor_data)
        
        # Small sleep to prevent overwhelming the queue
        time.sleep(0.1)

# Plotting setup
def setup_live_plot(motors_to_plot):
    """
    Set up a live plot for motor data
    
    :param motors_to_plot: List of motor names to plot
    :return: Figure and axes for live plotting
    """
    plt.ion()  # Turn on interactive mode
    fig, axes = plt.subplots(len(motors_to_plot), 1, figsize=(10, 3*len(motors_to_plot)))
    
    # Ensure axes is always a list, even for single plot
    if len(motors_to_plot) == 1:
        axes = [axes]
    
    # Initialize data storage for each motor
    motor_data_history = {motor: {'output_rpm': [], 'time': []} for motor in motors_to_plot}
    
    return fig, axes, motor_data_history

def update_live_plot(frame, motors_to_plot, axes, motor_data_history):
    """
    Update the live plot with new data
    
    :param frame: Animation frame (automatically managed)
    :param motors_to_plot: List of motor names to plot
    :param axes: Matplotlib axes
    :param motor_data_history: Dictionary to store historical data
    """
    # Collect all available data from the queue
    while not motor_data_queue.empty():
        motor_data = motor_data_queue.get()
        current_time = time.time()
        
        # Update data for each motor
        for motor_name in motors_to_plot:
            if motor_name in motor_data:
                motor_data_history[motor_name]['output_rpm'].append(motor_data[motor_name]['output_rpm'])
                motor_data_history[motor_name]['time'].append(current_time)
    
    # Trim old data (keep last 100 points)
    for motor_name in motors_to_plot:
        if len(motor_data_history[motor_name]['output_rpm']) > 100:
            motor_data_history[motor_name]['output_rpm'] = motor_data_history[motor_name]['output_rpm'][-100:]
            motor_data_history[motor_name]['time'] = motor_data_history[motor_name]['time'][-100:]
    
    # Clear and replot for each motor
    for i, motor_name in enumerate(motors_to_plot):
        axes[i].clear()
        axes[i].plot(motor_data_history[motor_name]['time'], 
                     motor_data_history[motor_name]['output_rpm'])
        axes[i].set_title(f'{motor_name} Output RPM')
        axes[i].set_xlabel('Time')
        axes[i].set_ylabel('RPM')
    
    plt.tight_layout()

# Rest of the existing simulation setup remains the same as in previous script
# (AutoSim model generation, motor initialization, etc.)

# Main script
def main():
    # Call AutoSim to generate the new robot spec:
    model_config_path = os.getcwd() + '/model_configs/Test/model_config.yaml'
    motor_config_path = os.getcwd() + '/motor_configs/frameless.yaml'
    motor_config = yaml.safe_load(Path(motor_config_path).read_text())

    # Generate the new robot spec:
    walter = AutoSim.GenerateModel(model_config_path=model_config_path, motor_config_path=motor_config_path)

    # Add payload to walter body
    walter.add_payload(mass=32, body_loc=[0,0,0.2], size=[0.2, 0.1, 0.1])

    # Generate the scene around the robot (groundplane and sky)
    walter.gen_scene()

    # Add obstacles (same as before)
    walter.add_stairs(rise=0.2, run=0.3, num_steps=15)
    walter.add_log(d=0.4, length=2)
    walter.add_incline(angle_deg=40, pos=[3, 5, 0], width=1.5, length=4)
    walter.add_box(pos=[5.5, 5, 2.5], size=[1, 2, 0.1])
    walter.add_incline(angle_deg=-40, pos=[8, 5, 0], width=1.5, length=4)

    # Compile the model:
    m = walter.spec.compile()
    d = mujoco.MjData(m)

    # Initialize motors (similar to previous script)
    fr_hip = motor.MotorModel(m, d, 'head_right_thigh_joint', motor_config['hip_params'], 12)
    fl_hip = motor.MotorModel(m, d, 'head_left_thigh_joint',  motor_config['hip_params'], 8)
    br_hip = motor.MotorModel(m, d, 'torso_right_thigh_joint',  motor_config['hip_params'], 4)
    bl_hip = motor.MotorModel(m, d, 'torso_left_thigh_joint',  motor_config['hip_params'], 0)

    br_wheel1_joint = motor.MotorModel(m, d, 'torso_right_shin_front_wheel_joint', motor_config['wheel_params'], 6)

    motors = [br_wheel1_joint, br_hip]  # Modify as needed

    # Initialize joystick controller
    controller = js_ctrl.JoystickController("logitech", m, d, motors)

    # Set up motors to log and plot
    motors_to_log = motors
    motors_to_plot = [motor for motor in motors_to_log]

    # Set up live plotting
    fig, axes, motor_data_history = setup_live_plot(motors_to_plot)

    # Start data collection thread
    data_collection_thread = threading.Thread(target=collect_motor_data, args=(motors_to_log,), daemon=True)
    data_collection_thread.start()

    # Create animation for live plotting
    ani = FuncAnimation(fig, update_live_plot, 
                        fargs=(motors_to_plot, axes, motor_data_history),
                        interval=100)

    # Main simulation loop:
    try:
        with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
            while viewer.is_running():
                step_start = time.time()

                # Step the simulation forward
                mujoco.mj_step(m, d)

                # Call joystick controller:
                controller.control(m, d)

                # Log motor data (optional, if needed)
                for motor_obj in motors_to_log:
                    motor_obj.log_data()

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()
                plt.pause(0.001)  # Small pause to update plot

                # Rudimentary time keeping, will drift relative to wall clock.
                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    except KeyboardInterrupt:
        # Set the stop event to signal the data collection thread to exit
        stop_event.set()
        # Wait for the data collection thread to finish
        data_collection_thread.join()
    finally:
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()