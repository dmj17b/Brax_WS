import mujoco

# Test sensor creation
spec = mujoco.MjSpec()

# Create a simple body hierarchy: worldbody -> parent -> child
parent = spec.worldbody.add_body(name='parent_body', pos=[0, 0, 0])
parent.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.1, 0.1, 0.1])

child = parent.add_body(name='child_body', pos=[0, 0, 0.2])
child.add_joint(type=mujoco.mjtJoint.mjJNT_HINGE, name='test_joint', axis=[0, 1, 0])
child.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.05, 0.05])

# Add a site to the child body
site = child.add_site(name='test_site', pos=[0, 0, 0])

# Try different sensor types that might work
sensor_types = [
    ('mjSENS_TORQUE', mujoco.mjtSensor.mjSENS_TORQUE),
    ('mjSENS_FORCE', mujoco.mjtSensor.mjSENS_FORCE),
    ('mjSENS_JOINTPOS', mujoco.mjtSensor.mjSENS_JOINTPOS),
]

for name, sensor_type in sensor_types:
    try:
        sensor = spec.add_sensor(
            name=f'test_{name}',
            type=sensor_type,
            objname='test_site' if 'FORCE' in name or 'TORQUE' in name else 'test_joint'
        )
        print(f"Sensor {name} added successfully")
    except Exception as e:
        print(f"Error adding sensor {name}: {e}")

# Compile to check
try:
    model = spec.compile()
    print("Model compiled successfully")
except Exception as e:
    print(f"Compilation error: {e}")
