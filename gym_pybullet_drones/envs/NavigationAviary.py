import numpy as np

from gym_pybullet_drones.envs.BaseRLAviary import BaseRLAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType

import pybullet as p

class HoverAviary(BaseRLAviary):
    """Single agent RL problem: hover at position."""

    ################################################################################
    
    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics=Physics.PYB,
                 pyb_freq: int = 240,
                 ctrl_freq: int = 30,
                 gui=False,
                 record=False,
                 obs: ObservationType=ObservationType.KIN,
                 act: ActionType=ActionType.RPM
                 ):
        """Initialization of a single agent RL environment.

        Using the generic single agent RL superclass.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        initial_xyzs: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
        initial_rpys: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
        physics : Physics, optional
            The desired implementation of PyBullet physics/custom dynamics.
        pyb_freq : int, optional
            The frequency at which PyBullet steps (a multiple of ctrl_freq).
        ctrl_freq : int, optional
            The frequency at which the environment steps.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        record : bool, optional
            Whether to save a video of the simulation.
        obs : ObservationType, optional
            The type of observation space (kinematic information or vision)
        act : ActionType, optional
            The type of action space (1 or 3D; RPMS, thurst and torques, or waypoint with PID control)

        """
        self.obstacle_min_radius = 5.0
        self.obstacle_max_radius = 35.0
        self.num_obstacle = 4
        self._generateTarget()
        self.EPISODE_LEN_SEC = 120
        super().__init__(drone_model=drone_model,
                         num_drones=1,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         pyb_freq=pyb_freq,
                         ctrl_freq=ctrl_freq,
                         gui=gui,
                         record=record,
                         obs=obs,
                         act=act
                         )

    ################################################################################
    def reset(self, seed=None, options=None):
        if seed:
            np.random.seed(seed)

        self._generateTarget()
        return super().reset(seed=seed, options=options)

    def _addObstacles(self):
        
        ## add danger zones
        obstacle_col_shape = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE,
            radius=5,
            physicsClientId=self.CLIENT
        )
        obstacle_vis_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=5,
            rgbaColor=[1.0, 0.0, 0.0, 1.0],  # RED
            physicsClientId=self.CLIENT
        )

        self.OBSTACLE_IDS = []

        for _ in range(self.num_obstacles):
            r = np.random.uniform(self.obstacle_min_radius, self.obstacle_max_radius)
            theta = np.random.uniform(0.0, 2.0 * np.pi)

            x = r * np.cos(theta)
            y = r * np.sin(theta)
            
            z = np.random.uniform(5, 15)
            pos = [x, y, z]
            orn = p.getQuaternionFromEuler([0.0, 0.0, 0.0])

            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=obstacle_col_shape,
                baseVisualShapeIndex=obstacle_vis_shape,
                basePosition=pos,
                baseOrientation=orn,
                physicsClientId=self.CLIENT
            )
            self.OBSTACLE_IDS.append(body_id)


        
        ## Add target
        target_col_shape = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE,
            radius=1,
            physicsClientId=self.CLIENT
        )
        target_vis_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=1,
            rgbaColor=[1.0, 1.0, 0.0, 1.0],  # YELLOW
            physicsClientId=self.CLIENT
        )

        target_pos = self.TARGET_POS.tolist()
        target_orn = p.getQuaternionFromEuler([0.0, 0.0, 0.0])

        self.TARGET_ID = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=target_col_shape,
            baseVisualShapeIndex=target_vis_shape,
            basePosition=target_pos,
            baseOrientation=target_orn,
            physicsClientId=self.CLIENT
        )

    def __checkCollision(self):

        drone_id = self.DRONE_IDS[0]

        # contacts obstacles 
        for obs_id in self.OBSTACLE_IDS:
            contacts = p.getContactPoints(
                bodyA=drone_id,
                bodyB=obs_id,
                physicsClientId=self.CLIENT
            )
            if len(contacts) > 0:
                self.collision_with_obstacle = True
                break
        
        # contacts target 
        contacts = p.getContactPoints(
            bodyA=drone_id,
            bodyB=self.TARGET_ID,
            physicsClientId=self.CLIENT
        )

        if len(contacts) > 0:
            self.collision_with_target = True
    
    def _generateTarget(self):
        """Sample a target 50 m away in a random direction, height 10 m."""
        r = 50.0
        theta = np.random.uniform(0.0, 2.0*np.pi)

        x = r*np.cos(theta)
        y = r*np.sin(theta)
        z = 10.0

        self.TARGET_POS = np.array([x, y, z], dtype=float)

    def _computeReward(self):
        """Computes the current reward value.

        Returns
        -------
        float
            The reward.

        """
        self.__checkCollision()

        ## reward is based off the inverse distances
        state = self._getDroneStateVector(0)
        dist = np.linalg.norm(self.TARGET_POS-state[0:3])
        ret = 1 / ( 1+ ret**2 )

        if (self.collision_with_target):
            ret += 10

        if (self.collision_with_obstacle):
            ret -+ 10

        return ret

    ################################################################################
    
    def _computeTerminated(self):
        """Computes the current done value.

        Returns
        -------
        bool
            Whether the current episode is done.

        """
        state = self._getDroneStateVector(0)
        if np.linalg.norm(self.TARGET_POS-state[0:3]) < .0001 :
            return True
        
        if self.collision_with_target or self.collision_with_obstacle:
            return True
        
        return False
        
    ################################################################################
    
    def _computeTruncated(self):
        """Computes the current truncated value.

        Returns
        -------
        bool
            Whether the current episode timed out.

        """
        state = self._getDroneStateVector(0)

        # end if the drone is too tited or time limited exceeded 
        if (abs(state[7]) > .4 or abs(state[8]) > .4): 
            return True
        if self.step_counter/self.PYB_FREQ > self.EPISODE_LEN_SEC:
            return True
        else:
            return False

    ################################################################################
    
    def _computeInfo(self):
        """Computes the current info dict(s).

        Unused.

        Returns
        -------
        dict[str, int]
            Dummy value.

        """
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years
