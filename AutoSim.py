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

        # Add Torso to World Body:
        torso_body = spec.worldbody.add_body(
            name='torso',
            pos=[0, 0, 0],
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
            },
            'shin': {
                'body_pos': np.array([0, shin_width / 2, -thigh_length]) + shin_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_CAPSULE,
                'geom_size': np.array([shin_width / 2, shin_length / 2, 0]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 0, 1, 0]),
                'mass': shin_mass,
            },
            'front_wheel': {
                'body_pos': np.array([shin_length / 2, wheel_width / 2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
            },
            'rear_wheel': {
                'body_pos': np.array([-shin_length / 2, wheel_width / 2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
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
                )
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
            },
            'shin': {
                'body_pos': np.array([0, shin_width / 2, -thigh_length]) + shin_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_CAPSULE,
                'geom_size': np.array([shin_width / 2, shin_length / 2, 0]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 0, 1, 0]),
                'mass': shin_mass,
            },
            'front_wheel': {
                'body_pos': np.array([shin_length / 2, wheel_width / 2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
            },
            'rear_wheel': {
                'body_pos': np.array([-shin_length / 2, wheel_width / 2, 0]) + wheel_offset,
                'body_quat': np.array([1, 0, 0, 0]),
                'geom_type': mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                'geom_size': np.array([wheel_radius, wheel_radius, wheel_width]),
                'geom_pos': np.array([0, 0, 0]),
                'geom_quat': np.array([1, 1, 0, 0]),
                'mass': wheel_mass,
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
                )
                body.add_geom(
                    name=geom_name,
                    type=head_children_params[child]['geom_type'],
                    size=head_children_params[child]['geom_size'],
                    pos=mirror * head_children_params[child]['geom_pos'],
                    quat=head_children_params[child]['geom_quat'],
                    mass=head_children_params[child]['mass'],
                )

        # Compile:
        self.mj_model = spec.compile()
        self.model_xml = spec.to_xml()


def main(argv=None):
    model_config_path = 'model_config.yaml'
    model_class = GenerateModel(model_config_path, None)

    xml_path = os.path.join(
        os.path.dirname(__file__),
        "autogen.xml",
    )

    with open(xml_path, "w") as f:
        f.writelines(model_class.model_xml)


if __name__ == '__main__':
    app.run(main)
