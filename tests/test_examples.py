def test_compare():
    from gym_pybullet_drones.examples.compare import run
    run(gui=False,plot=False,output_folder='tmp')

def test_downwash():
    from gym_pybullet_drones.examples.downwash import run
    run(gui=False,plot=False,output_folder='tmp')

def test_fly():
    from gym_pybullet_drones.examples.fly import run
    run(gui=False,plot=False,output_folder='tmp')

def test_groundeffect():
    from gym_pybullet_drones.examples.groundeffect import run
    run(gui=False,plot=False,output_folder='tmp')

def test_learn():
    from gym_pybullet_drones.examples.learn import run
    run(gui=False,plot=False,output_folder='tmp')

def test_velocity():
    from gym_pybullet_drones.examples.velocity import run
    run(gui=False,plot=False,output_folder='tmp')
