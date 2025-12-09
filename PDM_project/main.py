import numpy as np
import pkg_resources
import numpy as np
import pybullet as p
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from our_env import CustomCtrlAviary   # <- the custom env you defined earlier
from MPC import MPC_control


def main():
    # Create your custom environment
    env = CustomCtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        neighbourhood_radius=np.inf,
        physics=Physics.DYN,   # uses explicit dynamics (here we deciced what physics to use) We have two options: 
                               # either DYN which uses the method _dynamics or we can use PYB which uses the method physics +  _pybullet (p.stepsimulation). Both are defined in BaseAviary class.
        gui=True,              # IMPORTANT: show PyBullet GUI
        record=False,
        obstacles=True,        # IMPORTANT: calls your _addObstacles()
        user_debug_gui=True,
        output_folder='results'
    )
    ctrl = MPC_control(drone_model=DroneModel.CF2X)
    
    # 3. Define Your Path (RRT Planner output would go here)
    # List of [x, y, z] waypoints
    waypoints = [
        np.array([0.0, 0.0, 1.0]),  # Takeoff to 1m
        np.array([1.0, 0.0, 1.0]),  # Move X
        np.array([1.0, 1.0, 1.0]),  # Move Y
        np.array([0.0, 1.0, 1.0]),  # Back
        np.array([0.0, 0.0, 0.5])   # Land prepare
    ]
    
    current_wp_idx = 0
    wp_radius = 0.10 # 10cm accuracy required to switch
    
    # Get initial state
    obs, _ = env.reset()
    
    # --- SIMULATION LOOP ---
    for i in range(10000): # Run for some steps
        
        # A. Extract Current State from Observation
        # obs is usually [x, y, z, q1, q2, q3, q4, r, p, y, vx, vy, vz, wr, wp, wy]
        # Check BaseRLAviary._observationSpace docs for exact indices!
        # For CtrlAviary, it returns specific state directly usually.
        # Let's assume standard retrieval:
        
        # Get raw pybullet state for clarity (or parse 'obs')
        # [x, y, z], [qx, qy, qz, qw]
        pos, quat = p.getBasePositionAndOrientation(env.DRONE_IDS[0]) 
        vel, ang_vel = p.getBaseVelocity(env.DRONE_IDS[0])
        pos = np.array(pos)
        vel = np.array(vel)
        
        # --- B. WAYPOINT SWITCHING LOGIC (THE NAVIGATOR) ---
        target_pos = waypoints[current_wp_idx]
        
        # Calculate distance to current target
        distance = np.linalg.norm(pos - target_pos)
        
        # If we are close enough AND not at the last point, switch!
        if distance < wp_radius and current_wp_idx < len(waypoints) - 1:
            print(f"Reached waypoint {current_wp_idx}! Switching to {current_wp_idx + 1}")
            current_wp_idx += 1
            target_pos = waypoints[current_wp_idx]

        # --- C. CALL MPC ---
        # The MPC just tries to go to 'target_pos'
        action, _, _ = ctrl.computeControl(
            control_timestep=env.CTRL_TIMESTEP,
            cur_pos=pos,
            cur_quat=quat,
            cur_vel=vel,
            cur_ang_vel=ang_vel,
            target_pos=target_pos
            # target_vel=np.zeros(3) # Assume hover at waypoint
        )

        # D. Step Environment
        obs, reward, terminated, truncated, info = env.step(action)
        
        # Render/Sync
        env.render()
        p.stepSimulation()
        
    env.close()
    # Reset the environment to get the initial state
    #obs, info = env.reset()

    # Constant hover action: RPMs close to hover for each motor
    # Shape must be (NUM_DRONES, 4)
    hover_action = np.array([[env.HOVER_RPM, env.HOVER_RPM,
                              env.HOVER_RPM, env.HOVER_RPM]], dtype=np.float32)

    print("Environment running. Close the PyBullet window or press CTRL+C to exit.")

    try:
        while True:
            # Step the simulation with a constant hover command
            obs, reward, terminated, truncated, info = env.step(hover_action)
            print("caca")
            # You can call env.render() here, but with gui=True it's not needed
    except KeyboardInterrupt:
        print("Exiting simulation...")
    finally:
        env.close()


if __name__ == "__main__":
    main()