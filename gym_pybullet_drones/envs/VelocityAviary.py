import os
import numpy as np
from gym import spaces

from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics, BaseAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl

class VelocityAviary(BaseAviary):
    """Multi-drone environment class for high-level planning."""

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics=Physics.PYB,
                 freq: int=240,
                 aggregate_phy_steps: int=1,
                 gui=False,
                 record=False,
                 obstacles=False,
                 user_debug_gui=True,
                 goal_xyz= None,
                 collision_point = None,
                 protected_radius=None,
                 goal_radius = None, 
                 ):
        """Initialization of an aviary environment for or high-level planning.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        num_drones : int, optional
            The desired number of drones in the aviary.
        neighbourhood_radius : float, optional
            Radius used to compute the drones' adjacency matrix, in meters.
        initial_xyzs: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
        initial_rpys: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
        physics : Physics, optional
            The desired implementation of PyBullet physics/custom dynamics.
        freq : int, optional
            The frequency (Hz) at which the physics engine steps.
        aggregate_phy_steps : int, optional
            The number of physics steps within one call to `BaseAviary.step()`.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        record : bool, optional
            Whether to save a video of the simulation in folder `files/videos/`.
        obstacles : bool, optional
            Whether to add obstacles to the simulation.
        user_debug_gui : bool, optional
            Whether to draw the drones' axes and the GUI RPMs sliders.

        """
        #### Create integrated controllers #########################
        os.environ['KMP_DUPLICATE_LIB_OK']='True'
        if drone_model in [DroneModel.CF2X, DroneModel.CF2P]:
            self.ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for i in range(num_drones)]
        elif drone_model == DroneModel.HB:
            self.ctrl = [SimplePIDControl(drone_model=DroneModel.HB) for i in range(num_drones)]
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         freq=freq,
                         aggregate_phy_steps=aggregate_phy_steps,
                         gui=gui,
                         record=record,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui,                 
                         goal_xyz= goal_xyz,
                         collision_point = collision_point,
                         protected_radius= protected_radius,
                         goal_radius = goal_radius, 
                         )
        #### Set a limit on the maximum target speed ###############
        self.SPEED_LIMIT = 0.05*self.MAX_SPEED_KMH * (1000/3600)

    ################################################################################

    def _actionSpace(self):
        """Returns the action space of the environment.

        Returns
        -------
        ndarray
            A Box(4,) where the entry is a numpy array

        """
        #### Action vector ######### X       Y       Z   fract. of MAX_SPEED_KMH
        act_lower_bound = np.array([-1,     -1,     -1,                        0])
        act_upper_bound = np.array([ 1,      1,      1,                        1])
        #return spaces.Dict({str(i): spaces.Box(low=act_lower_bound,
        #                                       high=act_upper_bound,
        #                                       dtype=np.float32
        #                                       ) for i in range(self.NUM_DRONES)})

        return spaces.Box(low=act_lower_bound,
                          high=act_upper_bound,
                          dtype=np.float32)
    
    ################################################################################

    def _observationSpace(self):
        """Returns the observation space of the environment.

        Returns
        -------
        ndarray
            A box of shape Box{6+7*NUM_INTRUDERS,}

        """




        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ       P0            P1            P2            P3
        #obs_lower_bound = np.array([-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,           0.,           0.,           0.])
        #obs_upper_bound = np.array([np.inf,  np.inf,  np.inf, 1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM])
        #return spaces.Dict({str(i): spaces.Dict({"state": spaces.Box(low=obs_lower_bound,
        #                                                             high=obs_upper_bound,
        #                                                             dtype=np.float32
        #                                                             ),
        #                                         "neighbors": spaces.MultiBinary(self.NUM_DRONES)
        #                                         }) for i in range(self.NUM_DRONES)})


        #Hard coded for only 1 intruder 


        #observation vector           x         y     z         vx      vy       vz       relx    rely     relz     relvx    relvy    relvx  distance ownship to intruder     dist2goal
        obs_lower_bound = np.array([-np.inf, -np.inf, 0.,   -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,                               0.])
        obs_upper_bound = np.array([np.inf, np.inf, np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf, np.inf,                           np.inf])

        return spaces.Box(low=obs_lower_bound,
                          high=obs_upper_bound,
                          dtype=np.float32
                          )

    ################################################################################

    def _computeObs(self):
        """Returns the current observation of the environment.

        uses the ownship pos and velocities, relative positions and velocities wrt to the intruder and 
        distance2intruder as well as distance2goal

        TODO : Implement for multiple two intruders

        Returns
        -------
            ndarray of size 6 + 8*NUM_DRONES

        """




        rel_pos = self.pos[1,:]-self.pos[0,:]
        rel_vel = self.vel[1,:]-self.vel[0,:]
        doi = np.linalg.norm(rel_pos)
        d2g = np.linalg.norm(self.GOAL_XYZ-self.pos[0,:])

        #obs_vector = [ownship[0],ownship[1],ownship[2],ownship[10],ownship[11],ownship[12],rel_pos[0],rel_pos[0],rel_pos[1],rel_pos[2],rel_vel[0],rel_vel[1],rel_vel[2]]

        obs_vector = np.hstack([self.pos[0,:],self.vel[0,:],rel_pos,rel_vel,doi,d2g])
        return obs_vector.reshape(14)

        #adjacency_mat = self._getAdjacencyMatrix()
        #return {str(i): {"state": self._getDroneStateVector(i), "neighbors": adjacency_mat[i, :]} for i in range(self.NUM_DRONES)}

    ################################################################################

    def _preprocessAction(self,
                          action
                          ):
        """Pre-processes the action passed to `.step()` into motors' RPMs.

        Uses PID control to target a desired velocity vector.
        Converts a dictionary into a 2D array.

        Parameters
        ----------
        action : ndarray
            The desired velocity input for each drone, to be translated into RPMs.

        Returns
        -------
        ndarray
            (NUM_DRONES, 4)-shaped array of ints containing to clipped RPMs
            commanded to the 4 motors of each drone.

        """

        #TODO - Retreive the duration in self.
        
        INIT_VXVYVZ = ((self.GOAL_XYZ - self.INIT_XYZS)/(10)) / (np.linalg.norm(self.GOAL_XYZ - self.INIT_XYZS)/(10))
        INIT_VXVYVZ = np.hstack((INIT_VXVYVZ,0.99*np.ones((2,1))))

        #Keep only the intruder
        V_INTRUDER = np.delete(INIT_VXVYVZ,0,0)

        #Build action array : variable for the ownship (0th row) and fixed for the intruder (1 and onwards)
        action = np.vstack((action,V_INTRUDER))

        rpm = np.zeros((self.NUM_DRONES, 4))


        for k, v in enumerate(action):
            #### Get the current state of the drone  ###################
            state = self._getDroneStateVector(int(k))
            #### Normalize the first 3 components of the target velocity
            if np.linalg.norm(v[0:3]) != 0:
                v_unit_vector = v[0:3] / np.linalg.norm(v[0:3])
            else:
                v_unit_vector = np.zeros(3)
            temp, _, _ = self.ctrl[int(k)].computeControl(control_timestep=self.AGGR_PHY_STEPS*self.TIMESTEP, 
                                                    cur_pos=state[0:3],
                                                    cur_quat=state[3:7],
                                                    cur_vel=state[10:13],
                                                    cur_ang_vel=state[13:16],
                                                    target_pos=state[0:3], # same as the current position
                                                    target_rpy=np.array([0,0,state[9]]), # keep current yaw
                                                    target_vel=self.SPEED_LIMIT * np.abs(v[3]) * v_unit_vector # target the desired velocity vector
                                                    )
            rpm[int(k),:] = temp
        return rpm

    ################################################################################

    def _computeReward(self):
        """Computes the current reward value(s).

        Adds reward by evaluating the inverse of the distance to the goal 
        and substract rewards by the inverse of the distance to the intruder

        Returns
        -------
        int
            Dummy value.

        """
        
        rel_pos = self.pos[1,:]-self.pos[0,:]
        doi = np.linalg.norm(rel_pos)**2
        d2g = np.linalg.norm(self.GOAL_XYZ-self.pos[0,:])**2

        reward = 5/d2g #- 1/doi

        return reward 

    ################################################################################
    
    def _computeDone(self):
        """Computes the current done value(s).

        Checks if the ownship and the intruder is within the protected radius.
        Also checks if the ownship reached the goal

        Returns
        -------
        bool
            Whether the current episode is done.

        """

        #Checks for a collision
        for j in range(self.NUM_DRONES):
            ownship  = self._getDroneStateVector(int(0))
            intruder = self._getDroneStateVector(int(j))
        
            #Don't compute the distance between the ownship and itself
            if j==0:
                pass
            elif np.linalg.norm(ownship[0:3]-intruder[0:3])<self.PROTECTED_RADIUS:
                return True


        #Check if the goal has been reached
        if np.linalg.norm(ownship[0:3]-self.GOAL_XYZ)<self.GOAL_RADIUS:
            return True
        
        #Check if the ownship is on the ground
        if ownship[2]<0.1:
            return True


        #Check if the ownship is outside the domain
        if np.linalg.norm(ownship[0:3]-self.GOAL_XYZ) > 5:
            return True

        #Check for the length of the simulation
        if self.step_counter/self.SIM_FREQ > 15:
            return True
        else:
            return False

    ################################################################################
    
    def _computeInfo(self):
        """Computes the current info dict(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        dict[str, int]
            Dummy value.

        """
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years
