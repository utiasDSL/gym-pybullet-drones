import time
import numpy as np
import pybullet as p
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from new_env import CustomCtrlAviary
from MPC import MPC_control # Ensure file name matches your local file

def main():
    # 1. INIT ENV
    # Must use Physics.PYB if you want collisions with obstacles!
    env = CustomCtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        neighbourhood_radius=np.inf,
        physics=Physics.PYB,   # <--- FIX 1: PYB enables collisions. DYN ghosts through them.
        gui=True,              
        record=False,
        obstacles=True,        
        user_debug_gui=True
    )

    # 2. INIT CONTROLLER
    ctrl = MPC_control(drone_model=DroneModel.CF2X)

    # 3. RESET
    obs, _ = env.reset()
    
    # 4. DEFINE PATH
    waypoints = [
        np.array([0.0, 0.0, 1.0]),  # Hover
        np.array([1.0, 0.0, 1.0]),  # Move X
        np.array([1.0, 1.0, 1.0]),  # Move Y
        np.array([0.0, 1.0, 1.0]),  # Move X back
        np.array([0.0, 0.0, 1.0])   # Home
    ]
    
    current_wp_idx = 0
    wp_radius = 0.15 
    
    # 5. SYNC VARIABLES
    PYB_FREQ = env.PYB_FREQ # usually 240Hz
    CTRL_FREQ = env.CTRL_FREQ
    START = time.time()

    print("Starting MPC flight...")
    
    try:
        while True:
            # --- A. Get State (Ground Truth) ---
            # Stick with this method for Control/MPC, it is more robust for debugging.
            pos, quat = p.getBasePositionAndOrientation(env.DRONE_IDS[0]) 
            vel, ang_vel = p.getBaseVelocity(env.DRONE_IDS[0])
            
            pos = np.array(pos)
            vel = np.array(vel)
            ang_vel = np.array(ang_vel)
            
            # --- B. Waypoint Logic ---
            target_pos = waypoints[current_wp_idx]
            dist = np.linalg.norm(pos - target_pos)
            
            if dist < wp_radius and current_wp_idx < len(waypoints) - 1:
                print(f"Reached WP {current_wp_idx} -> Moving to WP {current_wp_idx+1}")
                current_wp_idx += 1
            
            # --- C. Compute Control ---
            # Note: We pass 0 as target velocity implies "Stop at Waypoint" behavior.
            # If you want smooth passing, you'd calculate a target_vel vector here.
            action, _, _ = ctrl.computeControl(
                control_timestep=env.CTRL_TIMESTEP,
                cur_pos=pos,
                cur_quat=quat,
                cur_vel=vel,
                cur_ang_vel=ang_vel,
                target_pos=target_pos
            )

            # --- D. Step Environment ---
            # env.step automatically calls p.stepSimulation() if physics=PYB
            reshaped_action = np.array([action])
            obs, reward, terminated, truncated, info = env.step(reshaped_action)
            
            # --- E. Render & Sync ---
            env.render()
            
            # Sync logic to match Wall-Clock time (makes it look realistic)
            now = time.time()
            # Calculate when the current step *should* finish
            expected_time = START + (env.step_counter / PYB_FREQ)
            if now < expected_time:
                time.sleep(expected_time - now)
        
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        env.close()

if __name__ == "__main__":
    main()