import numpy as np
from dataclasses import dataclass, field
from gym_pybullet_drones.envs.BaseRLAviary import BaseRLAviary
from gym_pybullet_drones.envs.NewBaseRLAviary import NewBaseRLAviary
from gym_pybullet_drones.examples.USV_trajectory import UsvTrajectory
from gym_pybullet_drones.examples.gradient_descent import LossFunction
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType


@dataclass(frozen=True)
class TimeData:
    T: float  # длительность во времени
    fs: int  # частота дискретизации
    n: int = field(init=False)  # число отсчетов
    dt: float = field(init=False)  # длительность отсчета времени
    t: np.ndarray = field(init=False)  # отсчеты времени

    def __post_init__(self):
        object.__setattr__(self, 'n', int(self.T * self.fs))
        object.__setattr__(self, 'dt', 1 / self.fs)
        object.__setattr__(self, 't', np.arange(self.n) * self.dt)

    def sample(self, fs):
        return TimeData(T=self.T, fs=fs)


class RlHoverAviary(NewBaseRLAviary):
    """Multi-agent RL problem: leader-follower."""

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel = DroneModel.CF2X,
                 num_drones: int = 2,
                 neighbourhood_radius: float = np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics = Physics.PYB,
                 pyb_freq: int = 300,
                 ctrl_freq: int = 20,
                 gui=False,
                 record=False,
                 obs: ObservationType = ObservationType.KIN,
                 act: ActionType = ActionType.PID,
                 ):
        """Initialization of a multi-agent RL environment.

        Using the generic multi-agent RL superclass.

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
        self.EPISODE_LEN_SEC = 30
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
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
        self.TARGET_POS = self.INIT_XYZS + np.array([[30, 0, 1 / (i + 1)] for i in range(num_drones)])

        r1 = np.array([[0, 0], [0, 20], [0, 40], [0, 60]])
        xyz1 = np.array([[0, 0, 0], [0, 20, 0], [0, 40, 0], [0, 60, 0]])
        time_data = TimeData(self.EPISODE_LEN_SEC, pyb_freq)
        self.trajs = UsvTrajectory(time_data, m=4, r0=r1, xyz0=xyz1)
        self.usv_coord = self.trajs.xyz

    ################################################################################

    def _computeReward(self):
        """Computes the current reward value.

        Returns
        -------
        float
            The reward.

        """
        #ret = 0

        gamma = 0.9
        #for i in range(self.usv_coord.shape[0]):
        states = np.array([self._getDroneStateVector(i) for i in range(self.NUM_DRONES)])
        uav_coord = np.transpose(np.array([states[:, 0], states[:, 1], states[:, 2]]), (1, 0))
        a = self.step_counter
        val = LossFunction.communication_quality_function(uav_coord.reshape(1, 2, 3),self.usv_coord[self.step_counter, :, :].reshape(1, 4, 3))
        ret = (10000 / val)

        return ret

    ################################################################################

    def _computeTerminated(self):
        """Computes the current done value.

        Returns
        -------
        bool
            Whether the current episode is done.

        """
        #states = np.array([self._getDroneStateVector(i) for i in range(self.NUM_DRONES)])
        #dist = 0
        #for i in range(self.NUM_DRONES):
            #dist += np.linalg.norm(self.TARGET_POS[i, :] - states[i][0:3])
        #if dist < .0001:
            #return True
        #else:
        return False

    ################################################################################

    def _computeTruncated(self):
        """Computes the current truncated value.

        Returns
        -------
        bool
            Whether the current episode timed out.

        """

        if self.step_counter / self.PYB_FREQ == self.EPISODE_LEN_SEC-1:
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
        return {"answer": 42}  #### Calculated by the Deep Thought supercomputer in 7.5M years
