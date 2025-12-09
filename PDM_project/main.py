import numpy as np

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from our_env import CustomCtrlAviary   # <- the custom env you defined earlier


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

    # Reset the environment to get the initial state
    obs, info = env.reset()

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