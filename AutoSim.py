from typing import Any
from absl import app
import os
from pathlib import Path
import yaml
import numpy as np
import mujoco


class GenerateModel():
    def __init__(
        self,
        model_config_path: Any,
        motor_config_path: Any,
    ) -> None:
        # Build model using Mujoco Spec:
        spec = mujoco.MjSpec()

        # Parse Configs:
        model_config = yaml.safe_load(Path(model_config_path).read_text())


        # Torso Params:
        torso_length = model_config['torso_params']['length']
        torso_width = model_config['torso_params']['width']
        torso_height = model_config['torso_params']['height']
        torso_mass = model_config['torso_params']['mass']
        torso_start_pos = model_config['torso_params']['start_pos']

        # Head Params:
        head_length = model_config['head_params']['length']
        head_width = model_config['head_params']['width']
        head_height = model_config['head_params']['height']
        head_mass = model_config['head_params']['mass']
        head_offset = np.asarray(model_config['head_params']['offset'])

        # Thigh Params:
        thigh_length = model_config['thigh_params']['length']
        thigh_width = model_config['thigh_params']['width']
        thigh_mass = model_config['thigh_params']['mass']
        thigh_torso_offset = np.asarray(model_config['thigh_params']['torso_offset'])
        thigh_head_offset = np.asarray(model_config['thigh_params']['head_offset'])

        # Shin Params:
        shin_length = model_config['shin_params']['length']
        shin_width = model_config['shin_params']['width']
        shin_mass = model_config['shin_params']['mass']
        shin_offset = np.asarray(model_config['shin_params']['offset'])

        # Wheel Params:
        wheel_radius = model_config['wheel_params']['radius']
        wheel_width = model_config['wheel_params']['width']
        wheel_mass = model_config['wheel_params']['mass']
        wheel_offset = np.asarray(model_config['wheel_params']['offset'])
        wheel_friction = np.asarray(model_config['wheel_params']['friction'])
        wheel_solref = np.asarray(model_config['wheel_params']['solref'])

        motor_config = yaml.safe_load(Path(motor_config_path).read_text())

        waist_spring_stiffness = motor_config['waist_params']['spring_stiffness']
        waist_damping = motor_config['waist_params']['damping']
        waist_range = motor_config['waist_params']['range']

        hip_kp = motor_config['hip_params']['Kp']
        hip_kd = motor_config['hip_params']['Kd']
        hip_gear_ratio = motor_config['hip_params']['gear_ratio']
        hip_stall_torque = motor_config['hip_params']['stall_torque']
        hip_no_load_speed = motor_config['hip_params']['no_load_speed']
        hip_rotor_inertia = motor_config['hip_params']['rotor_inertia']
        hip_damping = motor_config['hip_params']['damping']

        hip_armature = hip_rotor_inertia*hip_gear_ratio**2

        knee_kp = motor_config['knee_params']['Kp']
        knee_kd = motor_config['knee_params']['Kd']
        knee_gear_ratio = motor_config['knee_params']['gear_ratio']
        knee_stall_torque = motor_config['knee_params']['stall_torque']
        knee_no_load_speed = motor_config['knee_params']['no_load_speed']
        knee_rotor_inertia = motor_config['knee_params']['rotor_inertia']
        knee_damping = motor_config['knee_params']['damping']

        knee_armature = knee_rotor_inertia*knee_gear_ratio**2


        wheel_kp = motor_config['wheel_params']['Kp']
        wheel_kd = motor_config['wheel_params']['Kd']
        wheel_gear_ratio = motor_config['wheel_params']['gear_ratio']
        wheel_stall_torque = motor_config['wheel_params']['stall_torque']
        wheel_no_load_speed = motor_config['wheel_params']['no_load_speed']
        wheel_rotor_inertia = motor_config['wheel_params']['rotor_inertia']
        wheel_damping = motor_config['wheel_params']['damping']


        wheel_armature = wheel_rotor_inertia*wheel_gear_ratio**2


        # Add Torso to World Body:
        torso_body = spec.worldbody.add_body(
            name='torso',
            pos=torso_start_pos,
            quat=[1, 0, 0, 0],
        )
        torso_body.add_joint(
            type=mujoco.mjtJoint.mjJNT_FREE,
            name='torso_joint',
        )
        torso_body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[
                torso_length / 2, torso_width / 2, torso_height / 2,
            ],
            mass=torso_mass,
        )

        # Torso Kinematic Chain:
        parents = ['torso', 'thigh', 'shin', 'shin']
        children = ['thigh', 'shin', 'front_wheel', 'rear_wheel']
        side = ['left', 'right']
        mirror = [np.array([1, 1, 1]), np.array([1, -1, 1])]
        torso_children_params = {
            'thigh': {
                'body_pos': np.array([0, torso_width / 2, 0]) + thigh_torso_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_CAPSULE,
                'geom_size': np.array([thigh_width / 2, thigh_length / 2, 0]),
                'geom_pos': np.array([0, thigh_width / 2, -thigh_length / 2]),
                'geom_quat': np.array([1, 0, 0, 0]),
                'mass': thigh_mass,
                'armature': hip_armature,
                'damping': hip_damping,
            },
            'shin': {
                'body_pos': np.array([0, shin_width / 2 + thigh_width, -thigh_length]) + shin_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_CAPSULE,
                'geom_size': np.array([shin_width / 2, shin_length / 2, 0]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 0, 1, 0]),
                'mass': shin_mass,
                'armature': knee_armature,
                'damping': knee_damping,
            },
            'front_wheel': {
                'body_pos': np.array([shin_length / 2, wheel_width / 2 + shin_width/2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width/2]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
                'armature': wheel_armature,
                'damping': wheel_damping,
            },
            'rear_wheel': {
                'body_pos': np.array([-shin_length / 2, wheel_width / 2 + shin_width/2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width/2]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
                'armature': wheel_armature,
                'damping': wheel_damping,
            },
        }

        # Torso Children:
        for side, mirror in zip(side, mirror):
            for parent, child in zip(parents, children):
                if parent != 'torso':
                    parent_name = f'torso_{side}_{parent}'
                    body_name = f'torso_{side}_{child}'
                    joint_name = f'torso_{side}_{parent}_{child}_joint'
                else:
                    parent_name = parent
                    body_name = f'torso_{side}_{child}'
                    joint_name = f'{parent}_{side}_{child}_joint'
                geom_name = f'{body_name}_geom'
                parent_body = spec.find_body(parent_name)
                body = parent_body.add_body(
                    name=body_name,
                    pos=mirror * torso_children_params[child]['body_pos'],
                    quat=torso_children_params[child]['body_quat'],
                )
                body.add_joint(
                    type=mujoco.mjtJoint.mjJNT_HINGE,
                    name=joint_name,
                    axis=[0, 1, 0],
                    armature = torso_children_params[child]['armature'],
                    damping = torso_children_params[child]['damping'],
                )
                if(child == 'front_wheel' or child == 'rear_wheel'):
                    body.add_geom(
                        name=geom_name,
                        type=torso_children_params[child]['geom_type'],
                        size=torso_children_params[child]['geom_size'],
                        pos=mirror * torso_children_params[child]['geom_pos'],
                        quat=torso_children_params[child]['geom_quat'],
                        mass=torso_children_params[child]['mass'],
                        friction = wheel_friction,
                        solref = wheel_solref,
                    )
                else:
                    body.add_geom(
                        name=geom_name,
                        type=torso_children_params[child]['geom_type'],
                        size=torso_children_params[child]['geom_size'],
                        pos=mirror * torso_children_params[child]['geom_pos'],
                        quat=torso_children_params[child]['geom_quat'],
                        mass=torso_children_params[child]['mass'],
                    )


        # Add Head to Torso:
        head_position = np.asarray([
            (head_length / 2) + (torso_length / 2), 0, 0] + head_offset,
        )
        joint_position = -np.asarray([
            (head_length / 2), 0, 0
        ])
        head_body = torso_body.add_body(
            name='head',
            pos=head_position,
            quat=[1, 0, 0, 0],
        )
        head_body.add_joint(
            type=mujoco.mjtJoint.mjJNT_HINGE,
            name='head_joint',
            pos=joint_position,
            axis=[1, 0, 0],
            springdamper = [waist_spring_stiffness, waist_damping],
            range = waist_range,
        )
        head_body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[head_length / 2, head_width / 2, head_height / 2],
            pos=[0, 0, 0],
            quat=[1, 0, 0, 0],
            mass=head_mass,
        )

        # Head Kinematic Chain:
        parents = ['head', 'thigh', 'shin', 'shin']
        children = ['thigh', 'shin', 'front_wheel', 'rear_wheel']
        side = ['left', 'right']
        mirror = [np.array([1, 1, 1]), np.array([1, -1, 1])]
        head_children_params = {
            'thigh': {
                'body_pos': np.array([0, head_width / 2, 0]) + thigh_head_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_CAPSULE,
                'geom_size': np.array([thigh_width / 2, thigh_length / 2, 0]),
                'geom_pos': np.array([0, thigh_width / 2, -thigh_length / 2]),
                'geom_quat': np.array([1, 0, 0, 0]),
                'mass': thigh_mass,
                'armature': hip_armature,
                'damping': hip_damping,
            },
            'shin': {
                'body_pos': np.array([0, shin_width / 2 + thigh_width, -thigh_length]) + shin_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_CAPSULE,
                'geom_size': np.array([shin_width / 2, shin_length / 2, 0]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 0, 1, 0]),
                'mass': shin_mass,
                'armature': knee_armature,
                'damping': knee_damping,
            },
            'front_wheel': {
                'body_pos': np.array([shin_length / 2, wheel_width / 2 + shin_width/2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width/2]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
                'armature': wheel_armature,
                'damping': wheel_damping,
            },
            'rear_wheel': {
                'body_pos': np.array([-shin_length / 2, wheel_width / 2 + shin_width/2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width/2]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
                'armature': wheel_armature,
                'damping': wheel_damping,
            },
        }

        # head Children:
        for side, mirror in zip(side, mirror):
            for parent, child in zip(parents, children):
                if parent != 'head':
                    parent_name = f'head_{side}_{parent}'
                    body_name = f'head_{side}_{child}'
                    joint_name = f'head_{side}_{parent}_{child}_joint'
                else:
                    parent_name = parent
                    body_name = f'head_{side}_{child}'
                    joint_name = f'{parent}_{side}_{child}_joint'
                geom_name = f'{body_name}_geom'
                parent_body = spec.find_body(parent_name)
                body = parent_body.add_body(
                    name=body_name,
                    pos=mirror * head_children_params[child]['body_pos'],
                    quat=head_children_params[child]['body_quat'],
                )
                body.add_joint(
                    type=mujoco.mjtJoint.mjJNT_HINGE,
                    name=joint_name,
                    axis=[0, 1, 0],
                    armature = head_children_params[child]['armature'],
                    damping = head_children_params[child]['damping'],
                )
                if(child == 'front_wheel' or child == 'rear_wheel'):
                    body.add_geom(
                        name=geom_name,
                        type=head_children_params[child]['geom_type'],
                        size=head_children_params[child]['geom_size'],
                        pos=mirror * head_children_params[child]['geom_pos'],
                        quat=head_children_params[child]['geom_quat'],
                        mass=head_children_params[child]['mass'],
                        friction = wheel_friction,
                        solref = wheel_solref,
                    )
                else:
                    body.add_geom(
                        name=geom_name,
                        type=head_children_params[child]['geom_type'],
                        size=head_children_params[child]['geom_size'],
                        pos=mirror * head_children_params[child]['geom_pos'],
                        quat=head_children_params[child]['geom_quat'],
                        mass=head_children_params[child]['mass'],
                    )

# Adding Actuators:
        # Back left leg
        spec.add_actuator(
            name='bl_hip',
            target='torso_left_thigh_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='bl_knee',
            target='torso_left_thigh_shin_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='bl_wheel1_joint',
            target='torso_left_shin_front_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='bl_wheel2_joint',
            target='torso_left_shin_rear_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )

        # Back right leg
        spec.add_actuator(
            name='br_hip',
            target='torso_right_thigh_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='br_knee',
            target='torso_right_thigh_shin_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='br_wheel1_joint',
            target='torso_right_shin_front_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='br_wheel2_joint',
            target='torso_right_shin_rear_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )

        # Front left leg
        spec.add_actuator(
            name='fl_hip',
            target='head_left_thigh_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='fl_knee',
            target='head_left_thigh_shin_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='fl_wheel1_joint',
            target='head_left_shin_front_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='fl_wheel2_joint',
            target='head_left_shin_rear_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )

        # Front right leg
        spec.add_actuator(
            name='fr_hip',
            target='head_right_thigh_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='fr_knee',
            target='head_right_thigh_shin_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='fr_wheel1_joint',
            target='head_right_shin_front_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )
        spec.add_actuator(
            name='fr_wheel2_joint',
            target='head_right_shin_rear_wheel_joint',
            trntype = mujoco.mjtTrn.mjTRN_JOINT,
        )



        # Compile:
        self.mj_model = spec.compile()
        self.model_xml = spec.to_xml()
        self.spec = spec


    def gen_scene(self):
        # Create ground plane texture/material
        ground = self.spec.add_texture(type = mujoco.mjtTexture.mjTEXTURE_2D,
                              name="ground_texture",
                              builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, 
                              width=200, 
                              height=200, 
                              rgb1=[0.5, 0.8, 0.9], 
                              rgb2=[0.5, 0.9, 0.8],
                              markrgb=[0.8, 0.8, 0.8])
        
        self.spec.add_material(name="groundplane",
                              texrepeat=[2, 2],
                              reflectance=0., 
                              ).textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'ground_texture'
        
        self.spec.worldbody.add_geom(
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[0, 0, 0.05],
            material="groundplane",
        )


        # Create skybox so background isn't just black
        self.spec.add_texture(type = mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                              builtin = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                                width = 300,
                                height = 300,
                                name="skybox")

        # Add an array of lights to the scene:
        for i in range(5):
            for j in range(5):
                self.spec.worldbody.add_light(
                    pos=[2*i, 2*j, 15],
                    dir=[0, 0, -1],
                    diffuse=[0.1, 0.1, 0.1],
                    specular=[0., 0., 0.],
                    directional=True,
                )
        

    def add_box(self, pos:list, size:list):
        self.spec.worldbody.add_body(pos=pos).add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=size)


    def add_stairs(self, pos: list = [2,0,0], rise: float = 0.1, run: float = 0.1, width: float=1.2, num_steps: int=5):
        for i in range(num_steps):
            self.add_box(
                pos=[pos[0]+i*run, pos[1], pos[2] + i*rise],
                size=[run, width, rise],
            )

def main(argv=None):
    model_config_path = 'model_config.yaml'
    motor_config_path = 'motor_config.yaml'
    model_class = GenerateModel(model_config_path, motor_config_path)

    xml_path = os.path.join(
        os.path.dirname(__file__),
        "autogen.xml",
    )

    with open(xml_path, "w") as f:
        f.writelines(model_class.model_xml)


if __name__ == '__main__':
    app.run(main)
