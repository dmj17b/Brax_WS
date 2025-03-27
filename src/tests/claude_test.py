import time
import mujoco
import mujoco.viewer
import numpy as np
import yaml
from pathlib import Path
import os
import sys

# Append paths (as in your original script)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir,)))

import lib.MotorModel as motor
import lib.JoystickControl as js_ctrl
import AutoSim

def main():
    # Load configurations (similar to your original script)
    model_config_path = os.getcwd() + '/model_configs/Test/model_config.yaml'
    motor_config_path = os.getcwd() + '/motor_configs/frameless.yaml'
    motor_config = yaml.safe_load(Path(motor_config_path).read_text())

    # Generate robot model
    walter = AutoSim.GenerateModel(model_config_path=model_config_path, motor_config_path=motor_config_path)
    walter.add_payload(mass=32, body_loc=[0,0,0.2], size=[0.2, 0.1, 0.1])
    walter.gen_scene()

    # Add obstacles
    walter.add_stairs(rise=0.2, run=0.3, num_steps=15)
    walter.add_log(d=0.4, length=2)
    walter.add_incline(angle_deg=40, pos=[3, 5, 0], width=1.5, length=4)
    walter.add_box(pos=[5.5, 5, 2.5], size=[1, 2, 0.1])
    walter.add_incline(angle_deg=-40, pos=[8, 5, 0], width=1.5, length=4)

    # Compile the model
    m = walter.spec.compile()
    d = mujoco.MjData(m)

    # Initialize motors
    br_hip = motor.MotorModel(m, d, 'torso_right_thigh_joint', motor_config['hip_params'], 4)
    br_wheel1_joint = motor.MotorModel(m, d, 'torso_right_shin_front_wheel_joint', motor_config['wheel_params'], 6)
    
    # Select motors to log
    br_hip.enable_logging()
    br_wheel1_joint.enable_logging()

    # Start the plotting thread
    motor.MotorModel.start_plotting()

    # Initialize joystick controller
    motors = [br_hip, br_wheel1_joint]
    controller = js_ctrl.JoystickController("logitech", m, d, motors)

    # Main simulation loop
    try:
        with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
            while viewer.is_running():
                step_start = time.time()

                # Step simulation
                mujoco.mj_step(m, d)

                # Control and log data
                controller.control(m, d)
                
                # Log data for selected motors
                for motor_obj in motors:
                    motor_obj.log_data()
                    motor_obj.update_plot()

                # Sync viewer
                viewer.sync()

                # Time keeping
                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    except KeyboardInterrupt:
        print("Simulation stopped by user")
    finally:
        # Stop plotting thread
        motor.MotorModel.stop_plotting()

if __name__ == "__main__":
    main()