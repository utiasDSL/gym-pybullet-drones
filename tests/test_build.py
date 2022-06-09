import unittest

class TestBuild(unittest.TestCase):

    def test_imports(self):
        from gym_pybullet_drones.control.BaseControl import BaseControl
        from gym_pybullet_drones.utils.enums import DroneModel
        import gym_pybullet_drones.envs.multi_agent_rl.BaseMultiagentAviary

    def test_compare(self):
        from gym_pybullet_drones.examples.compare import run
        run(gui=False,plot=False)
    def test_downwash(self):
        from gym_pybullet_drones.examples.downwash import run
        run(gui=False,plot=False)
    def test_fly(self):
        from gym_pybullet_drones.examples.fly import run
        run(gui=False,plot=False)
    def test_groundeffect(self):
        from gym_pybullet_drones.examples.groundeffect import run
        run(gui=False,plot=False)
    def test_learn(self):
        from gym_pybullet_drones.examples.learn import run
        run(gui=False,plot=False)
    def test_velocity(self):
        from gym_pybullet_drones.examples.velocity import run
        run(gui=False,plot=False)
    
    

if __name__ == '__main__':
    unittest.main()