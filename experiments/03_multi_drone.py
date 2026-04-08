"""
Experiment 3: Multi-Drone Formation
=====================================
Three drones hover in a triangle formation using PID control.
This shows multi-agent coordination -- each drone independently
controls itself but together they form a pattern.

Run with:
    conda activate drones
    python experiments/03_multi_drone.py
"""

import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

NUM_DRONES = 3

# --- Starting positions: drones on the ground in a line ---
INIT_XYZ = np.array([
    [0.0, 0.0, 0.05],
    [0.5, 0.0, 0.05],
    [0.25, 0.4, 0.05],
])

# --- Target positions: triangle formation at 1 meter height ---
TARGETS = np.array([
    [0.0,  0.0, 1.0],   # Drone 0: left
    [1.0,  0.0, 1.0],   # Drone 1: right
    [0.5,  0.8, 1.5],   # Drone 2: top (higher)
])

# --- Create environment ---
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=NUM_DRONES,
    initial_xyzs=INIT_XYZ,
    physics=Physics.PYB,
    gui=True,
    pyb_freq=240,
    ctrl_freq=240,
)

# --- Create a PID controller for EACH drone ---
controllers = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(NUM_DRONES)]

obs, info = env.reset(seed=42)

print("=" * 50)
print(f"EXPERIMENT 3: {NUM_DRONES}-Drone Triangle Formation")
print("=" * 50)
for i in range(NUM_DRONES):
    print(f"  Drone {i}: start={INIT_XYZ[i]} -> target={TARGETS[i]}")
print()
print("Watch: 3 drones rise and form a triangle!")
print("Press Ctrl+C to stop.")
print()

try:
    for step in range(12000):

        # --- Compute PID action for each drone ---
        actions = np.zeros((NUM_DRONES, 4))
        for i in range(NUM_DRONES):
            actions[i], _, _ = controllers[i].computeControlFromState(
                control_timestep=1/240,
                state=obs[i],
                target_pos=TARGETS[i],
                target_rpy=np.array([0, 0, 0]),
            )

        obs, reward, terminated, truncated, info = env.step(actions)

        # Print every 2 seconds
        if step % 480 == 0:
            print(f"[t={step/240:.1f}s]")
            for i in range(NUM_DRONES):
                pos = obs[i][0:3]
                error = np.linalg.norm(pos - TARGETS[i])
                print(f"  Drone {i}: pos=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})  "
                      f"error={error:.4f}m")
            print()

except KeyboardInterrupt:
    print("\nStopped by user.")

env.close()
print("Done!")
