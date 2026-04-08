"""
Experiment 1: Your First Flight
================================
This script creates a drone in a virtual world and applies random motor commands.
Watch what happens -- the drone will probably crash immediately!

Run with:
    conda activate drones
    python experiments/01_first_flight.py
"""

import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

# --- Create the environment ---
# gui=True means you'll SEE the drone in a 3D window
# drone_model: CF2X is the Crazyflie quadcopter
# physics: PYB means full PyBullet physics simulation
env = CtrlAviary(
    drone_model=DroneModel.CF2X,
    num_drones=1,
    physics=Physics.PYB,
    gui=True,                    # Set to False if no window appears
    pyb_freq=240,                # Physics runs at 240 Hz
    ctrl_freq=240,               # Control runs at 240 Hz
)

# --- Reset the environment ---
obs, info = env.reset(seed=42)   # <-- YOUR FIX makes this work!

print("=" * 50)
print("EXPERIMENT 1: Random Motor Commands")
print("=" * 50)
print(f"Observation shape: {obs.shape}")
print(f"Action space: {env.action_space}")
print(f"Starting position: x={obs[0][0]:.2f}, y={obs[0][1]:.2f}, z={obs[0][2]:.2f}")
print()
print("Watch the 3D window -- the drone will try to fly!")
print("Press Ctrl+C to stop.")
print()

# --- Run the simulation ---
try:
    for step in range(10000):  # 10,000 steps at 240Hz = ~42 seconds of flight

        # Generate random actions (this is what an untrained AI does)
        # Each action is 4 RPM values for the 4 motors
        # RPM range is roughly 0 to 21,700 for the Crazyflie
        action = env.action_space.sample()  # random motor commands

        # Step the simulation forward
        obs, reward, terminated, truncated, info = env.step(action)

        # Print status every 240 steps (= every 1 second of simulation)
        if step % 240 == 0:
            pos = obs[0][0:3]   # x, y, z position
            vel = obs[0][6:9]   # vx, vy, vz velocity
            print(f"[t={step/240:.1f}s] pos=({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f})  "
                  f"vel=({vel[0]:+.2f}, {vel[1]:+.2f}, {vel[2]:+.2f})  "
                  f"reward={reward[0]:+.4f}")

        if terminated or truncated:
            print(f"\n--- Episode ended at step {step} ({'crashed' if terminated else 'timeout'}) ---")
            obs, info = env.reset(seed=42)

except KeyboardInterrupt:
    print("\nStopped by user.")

env.close()
print("Done!")
