import mujoco
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('QtAgg')

class MotorModel:
  def __init__(self, m: mujoco.MjModel, d: mujoco.MjData, motor_name: str, motor_params: dict, ctrl_index: int):
    self.m = m
    self.d = d
    self.motor_name = motor_name
    self.Kp = motor_params['Kp']
    self.Kd = motor_params['Kd']
    self.gear_ratio = motor_params['gear_ratio']
    self.t_stall = motor_params['stall_torque']
    self.w_no_load = motor_params['no_load_speed']
    self.ctrl_index = ctrl_index
    self.target_pos = 0
    self.target_vel = 0
    self.target_torque = 0
    self.limited_torque = 0
    self.target_vel = 0
    self.tau_max = 0
    self.torques = []
    self.omegas = []
    self.target_positions = []
    self.positions = []
    self.velocities = []

  def debug(self):
    # print(f"Motor name: {self.motor_name}")
    # print(f"Kp: {self.Kp}")
    # print(f"Kd: {self.Kd}")
    # print(f"Target pos: {self.target_pos}")
    # print(f"Target vel: {self.target_vel}")
    # print(f"Target torque: {self.target_torque}")
    # print(f"Limited torque: {self.limited_torque}")
    # q = self.d.jnt(self.motor_name).qpos
    # qdot = self.d.jnt(self.motor_name).qvel
    # print(f"q: {q}")
    # print(f"qdot: {qdot}")
    if(abs(self.limited_torque)>=abs(self.tau_max)):
      print(f"Motor name: {self.motor_name}")
      print("TORQUE LIMIT REACHED")
      print(f"Target torque: {self.target_torque}")
      print(f"Limited torque: {self.limited_torque}")
      print(f"Angular velocity: {self.d.jnt(self.motor_name).qvel[0]}")

  # Position control function that limits torque according to speed torque curve
  def pos_control(self, target_pos: float):
    self.target_pos = target_pos
    q = self.d.jnt(self.motor_name).qpos
    qdot = self.d.jnt(self.motor_name).qvel
    tau = self.Kp*(self.target_pos - q) - self.Kd*qdot
    self.target_torque = tau
    self.limited_torque = self.speed_torque_limit(tau)
    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    return self.limited_torque
  
  # Velocity control function that limits torque according to speed torque curve
  def vel_control(self, target_vel: float):
    self.target_vel = target_vel
    qdot = self.d.jnt(self.motor_name).qvel
    tau = self.Kp*(self.target_vel - qdot) #+ self.Kd*(-self.d.jnt(self.motor_name).qacc)
    self.target_torque = tau
    self.limited_torque = self.speed_torque_limit(tau)

    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    return self.limited_torque
  
  def torque_control(self, target_torque: float):
    self.target_torque = target_torque
    self.limited_torque = self.speed_torque_limit(target_torque)
    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    return self.limited_torque


  # Function that returns the limited torque according to the speed torque curve
  def speed_torque_limit(self, target_torque: float):
    w_motor = abs(self.d.jnt(self.motor_name).qvel)*self.gear_ratio
    self.tau_max = -(self.t_stall/self.w_no_load)*w_motor + self.t_stall

    # If target torque is greater than speed/torque curve allows, limit it
    if(abs(target_torque) > abs(self.tau_max) and w_motor < abs(self.w_no_load)):
      return np.sign(target_torque)*self.tau_max
    # If angular velocity is greater than no load speed, limit torque to zero
    elif(w_motor >= abs(self.w_no_load)):
      return 0.0
    # Otherwise return the target torque
    else:
      return target_torque


  def log_data(self):
    self.torques.append(float(abs(self.limited_torque)))
    self.omegas.append(float(abs(self.d.jnt(self.motor_name).qvel[0])*self.gear_ratio))
    self.target_positions.append(self.target_pos)
    self.positions.append(self.d.jnt(self.motor_name).qpos[0])
    self.velocities.append(self.d.jnt(self.motor_name).qvel[0]*self.gear_ratio)

  def plot_speed_torque_curve(self):
    w_range = np.linspace(0, self.w_no_load, 1000)
    T_line = -self.t_stall/self.w_no_load*w_range + self.t_stall
    plt.title(f"{self.motor_name} Speed Torque Curve")
    plt.plot(self.omegas, self.torques, 'b*')
    plt.plot(w_range, T_line, 'r--')
    plt.xlabel('Angular Velocity (rad/s)')
    plt.ylabel('Torque (Nm)')
    plt.plot(w_range, T_line, 'r--')
    plt.show()

  def plot_positions(self):
    plt.title(f"{self.motor_name} Position Control")
    plt.plot(self.positions, 'b')
    plt.plot(self.target_positions, 'r')
    plt.xlabel('Time Step')
    plt.ylabel('Position (rad)')
    plt.show()