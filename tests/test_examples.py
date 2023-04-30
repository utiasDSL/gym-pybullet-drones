def test_api():
    from gym_pybullet_drones.examples.api import run
    run(gui=False, plot=False, output_folder='tmp')

def test_pid():
    from gym_pybullet_drones.examples.pid import run
    run(gui=False, plot=False, output_folder='tmp')

def test_pid_velocity():
    from gym_pybullet_drones.examples.pid_velocity import run
    run(gui=False, plot=False, output_folder='tmp')

def test_downwash():
    from gym_pybullet_drones.examples.downwash import run
    run(gui=False, plot=False, output_folder='tmp')

# Commented out until Stable Baselines 3 2.0 is released
# def test_learn():
#     from gym_pybullet_drones.examples.learn import run
#     run(gui=False, plot=False, output_folder='tmp')
