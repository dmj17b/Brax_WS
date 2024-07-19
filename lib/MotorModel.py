import mujoco
import numpy as np

class MotorModel:
  def __init__(self, m: mujoco.MjModel, d: mujoco.MjData, motor_name: str, motor_params: dict, ctrl_index: int):
    self.m = m
    self.d = d
    self.motor_name = motor_name
    self.Kp = motor_params['Kp']
    self.Kd = motor_params['Kd']
    self.gear_ratio = motor_params['gear_ratio']
    self.t_stall = motor_params['t_stall']
    self.w_no_load = motor_params['w_no_load']
    self.ctrl_index = ctrl_index
    self.target_pos = 0
    self.target_vel = 0
    self.target_torque = 0
    self.limited_torque = 0
    self.target_vel = 0
    self.tau_max = 0

  def debug(self):
    print(f"Motor name: {self.motor_name}")
    print(f"Kp: {self.Kp}")
    print(f"Kd: {self.Kd}")
    print(f"Target pos: {self.target_pos}")
    print(f"Target vel: {self.target_vel}")
    print(f"Target torque: {self.target_torque}")
    print(f"Limited torque: {self.limited_torque}")
    q = self.d.jnt(self.motor_name).qpos
    qdot = self.d.jnt(self.motor_name).qvel
    print(f"q: {q}")
    print(f"qdot: {qdot}")
    ...

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
  ## DOES NOT WORK YET##
  def vel_control(self, target_vel: float):
    self.target_vel = target_vel
    q = self.d.jnt(self.motor_name).qpos
    qdot = self.d.jnt(self.motor_name).qvel
    print(qdot)
    tau = self.Kp*(self.target_vel - qdot) + self.Kd*(-self.d.jnt(self.motor_name).qacc)
    self.target_torque = tau
    self.limited_torque = self.speed_torque_limit(tau)
    # print(self.target_vel-qdot)
    # self.d.ctrl[self.ctrl_index] = tau*self.gear_ratio
    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    return self.limited_torque
  
  def torque_control(self, target_torque: float):
    self.target_torque = target_torque
    self.limited_torque = self.speed_torque_limit(target_torque)
    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    print(self.limited_torque)
    return self.limited_torque


  # Function that returns the limited torque according to the speed torque curve
  def speed_torque_limit(self, target_torque: float):
    w_motor = abs(self.d.jnt(self.motor_name).qvel)*self.gear_ratio
    self.tau_max = -(self.t_stall/self.w_no_load)*w_motor + self.t_stall

    # If target torque is greater than speed/torque curve allows, limit it
    if(abs(target_torque) > abs(self.tau_max)):
      return np.sign(target_torque)*self.tau_max
    
    # Otherwise return the target torque
    else:
      return target_torque

