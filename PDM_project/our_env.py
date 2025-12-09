"""Custom CtrlAviary environment with specific obstacle layout."""
import pkg_resources
import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary

class CustomCtrlAviary(CtrlAviary):
    """CtrlAviary with my own obstacle layout."""

    def _addObstacles(self):
        """
        Override BaseAviary._addObstacles().
        This is called from _housekeeping() if self.OBSTACLES == True.
        """
        

        # Cube in front of origin
        p.loadURDF(
            "cube_no_rotation.urdf",
            [1.0, 0.0, 0.5],                # position [x, y, z]
            p.getQuaternionFromEuler([0,0,0]),
            physicsClientId=self.CLIENT
        )

        

        
        p.loadURDF(
            "cube_no_rotation.urdf",
            [1.0, 4.0, 0.0],
            p.getQuaternionFromEuler([0,0,0]),
            physicsClientId=self.CLIENT
        )
        
        p.loadURDF(
            "duck_vhacd.urdf",
            [-.5, -.5, .05],
            p.getQuaternionFromEuler([0, 0, 0]),
            physicsClientId=self.CLIENT
        )

        #Code taken from https://github.com/phuongboi/drone-racing-using-reinforcement-learning
        g1 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
                   [0, 1, 0],
                   p.getQuaternionFromEuler([0, 0, np.pi/2]),
                   physicsClientId=self.CLIENT
                   )
        g2 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
                   [0.5, 3, 0],
                   p.getQuaternionFromEuler([0, 0, np.pi/2]),
                   physicsClientId=self.CLIENT
                   )
        g3 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
                   [-0.5, 5, 0],
                   p.getQuaternionFromEuler([0, 0, np.pi/2]),
                   physicsClientId=self.CLIENT
                   )
        g4 = p.loadURDF(pkg_resources.resource_filename('gym_pybullet_drones', 'assets/gate.urdf'),
                   [-0.5, 6, 0],
                   p.getQuaternionFromEuler([0, 0, np.pi/2]),
                   physicsClientId=self.CLIENT
                   )

        self.GATE_IDs = np.array([g1, g2, g3, g4])
        
    '''
    def _preprocessAction(self,
                          action
                          ):
        """Pre-processes the action passed to .step() into motors' RPMs.

        It can be the same as in CtrlAviary or customized depending on the action format (the output of the MPC).

        Parameters
        ----------
        action : ndarray | dict[..]
            The input action for one or more drones, to be translated into RPMs.

        """
        #raise NotImplementedError
    '''
    '''
    Some comments:
    this class is an inheritance of CtrlAviary which is by itslef an inheritance of BaseAviary. It overrides the _addObstacles() method to define a custom layout of obstacles in the simulation environment.
    The other functions such as get_action_space(), get_observation_space() are inherited from the parent classes and do not need to be redefined here unless specific custom behavior is desired.
    However it can be noted that get_observation_space() is returning every observation (pos, vel, rot, ang vel) which for our linearized model is not necessary. So when using MPC we need to pick only the relevant states.
    We can still use the .step() fucntion and we pass the action (motor commands) computed by the MPC to it. The step function will take care of applying the action and returning the full observation, from which we can extract the relevant states for our model.
    In the step function a pre_process of the actions is done. Preporcess here means converting the output of the MPC to the format expected by the simulator (e.g. scaling motor commands). This is expressed in the method  named _physics that expects rpm as input.
    The output of the MPC can be choosen depending on the model and on what we are optimizing. For example we can output desired roll, pitch, yaw rate and thrust, and then convert these to motor commands before passing them to the step function. (we also clip the motor commands to be within the valid range).
    In the .step function the next state is computed using _physics method that applies the motor commands and simulates the drone dynamics for one time step + p.stepsimulation where the actual physics step is done.
    This last p.stepsimulation integrates the equations of motion with the current forces/torques and updates the state of the drone in the simulator.
    We can also use the dynamic method which use the dynamic model and compute everyintg in the same method (maybe more similar to dynamic model). We decide what to use when we make the instance of the class in main.py by choosing the physics parameter (either DYN or PYB).
    There are methods to store the state such as _updateAndStoreKinematicInformation() and _getDroneStateVector() that can be used to retrieve the current state of the drone after the step.
    Then to continue the loop of the MPC we extract the observations after updating them using computeObs() method.
    '''
