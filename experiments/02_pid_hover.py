"""
Experiment 2: PID-Controlled Hover
====================================
Instead of random commands, we use a PID controller to make the drone
hover at a target position. This is classical control -- no AI, just math.

Compare this to Experiment 1: the drone should hover perfectly!

Run with:
    conda activate drones
    python experiments/02_pid_hover.py
"""

import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# --- Create environment ---
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    physics=Physics.PYB,
    gui=True,
    pyb_freq=240,
    ctrl_freq=240,
)

# --- Create PID controller ---
# This is the classical controller -- it uses math, not learning
ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)

# --- Target position: hover at (0, 0, 1) = 1 meter above ground ---
TARGET = np.array([0.0, 0.0, 1.0])

obs, info = env.reset(seed=42)

print("=" * 50)
print("EXPERIMENT 2: PID Hover at 1 meter")
print("=" * 50)
print(f"Target position: {TARGET}")
print(f"Starting position: x={obs[0][0]:.2f}, y={obs[0][1]:.2f}, z={obs[0][2]:.2f}")
print()
print("Watch: the drone should smoothly rise and hover at 1 meter!")
print("Press Ctrl+C to stop.")
print()

try:
    for step in range(10000):

        # --- PID controller computes the motor RPMs ---
        # It reads the drone's current state and calculates what
        # motor speeds are needed to reach the target
        action, _, _ = ctrl.computeControlFromState(
            control_timestep=1/240,          # time per step
            state=obs[0],                     # current drone state
            target_pos=TARGET,                # where we want to be
            target_rpy=np.array([0, 0, 0]),  # no rotation target
        )

        # Step the simulation with the PID-computed action
        obs, reward, terminated, truncated, info = env.step(action.reshape(1, -1))

        # Print every second
        if step % 240 == 0:
            pos = obs[0][0:3]
            error = np.linalg.norm(pos - TARGET)  # distance to target
            print(f"[t={step/240:.1f}s] pos=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})  "
                  f"error={error:.4f}m")

        if terminated or truncated:
            print(f"\n--- Episode ended at step {step} ---")
            obs, info = env.reset(seed=42)

except KeyboardInterrupt:
    print("\nStopped by user.")

env.close()
print("Done!")
