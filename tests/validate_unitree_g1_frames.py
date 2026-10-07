"""Independent frame check against MuJoCo's object-velocity API, not another qvel echo."""
from pathlib import Path
import sys
import numpy as np
import mujoco
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'native'))
from unitree_g1_reference import Plant

plant = Plant('freebase')
# Declared static test setup only: no state injection in the browser or controller.
plant.data.qpos[3:7] = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
plant.data.qvel[3:6] = [1, 2, 3]
plant.declare_setup('frame-reference fixture', frame='pelvis-local', angularRadS=[1, 2, 3])
mujoco.mj_forward(plant.model, plant.data)
body = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
velocity = np.zeros(6)
mujoco.mj_objectVelocity(plant.model, plant.data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
state = plant.root()
assert state['frame'] == 'mujoco_world'
assert np.allclose(state['angularVelocityRadS'], velocity[:3], atol=1e-6)
assert not np.allclose(state['angularVelocityRadS'], plant.data.qvel[3:6])
print('G1 native angular velocity agrees with mj_objectVelocity(world):', state['angularVelocityRadS'])

# The native observer must publish on integer control ticks, not floating-time thresholds.
for dt in (.002, .001):
    plant = Plant('freebase', timestep=dt, control_interval=dt)
    command = list(plant.accepted)
    samples = plant.run(.06, lambda _: command)
    assert [s['simulationTimeSeconds'] for s in samples] == [.02, .04, .06]
print('G1 native observation cadence: exact 20 ms at both numerical settings')
