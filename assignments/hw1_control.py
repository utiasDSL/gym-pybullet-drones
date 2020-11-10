"""Control implementation for assignment 1.
Class HW1Control is used by the simulation in file "hw_simulation.py".

Example:
    To run the simulation, type in a terminal

        $ python hw1_simulation.py

Todo:
    * Implement method HW1Control.compute_control()
    * Tune the PID coefficients in HW1Control.__init__()
    * ..
"""
import numpy as np
from gym_pybullet_drones.envs.BaseAviary import BaseAviary

class HW1Control():
    """Control class for assignment 1.
    """

    def __init__(self, env: BaseAviary):
        """ Initialization of class HW1Control.

        Args:
            env (BaseAviary): the PyBullet-based simulation environment.
        """
        self.gravity = env.GRAVITY
        """float: gravity, the product of the drone's mass M by acc g."""
        self.timestep = env.TIMESTEP
        """float: simulation and control timestep."""
        self.kf_coeff = env.KF
        """float: RPMs to force coefficient."""
        self.km_coeff = env.KM
        """float: RPMs to torque coefficient."""
        ############################################################
        ############################################################
        #### HOMEWORK CODE (START) #################################
        ############################################################
        ############################################################
        self.p_coeff_position = None
        """proportional coefficient(s) for position control."""
        self.i_coeff_position = None
        """integral coefficient(s) for position control."""
        self.d_coeff_position = None
        """derivative coefficient(s) for position control."""
        self.p_coeff_attitude = None
        """proportional coefficient(s) for attitude control."""
        self.i_coeff_attitude = None
        """integral coefficient(s) for attitude control."""
        self.d_coeff_attitude = None
        """derivative coefficient(s) for attitude control."""
        ############################################################
        ############################################################
        #### HOMEWORK CODE (END) ###################################
        ############################################################
        ############################################################
        self.reset()

    def reset(self):
        """ Resets the controller counters and variables (integral errors and
        previous control step errors).
        """
        self.control_counter = 0
        self.last_position_error = np.zeros(3)
        self.integral_position_error = np.zeros(3)
        self.last_attitude_error = np.zeros(3)
        self.integral_attitude_error = np.zeros(3)

    def compute_control(self,
                        current_position,
                        current_quaternion,
                        current_velocity,
                        current_angular_velocity,
                        target_position,
                        target_velocity=np.zeros(3),
                        ):
        """Compute the propellers' RPMs for the target state, given the
        current state.

        Args:
            current_position ((3,) NumPy array): global x, y, z, in meters.
            current_quaternion ((4,) NumPy array): global, quaternion.
            current_velocity ((3,) NumPy array): global vx, vy, vz, in m/s.
            current_angular_velocity ((3,) NumPy array): global, in rad/s.
            target_position ((3,) NumPy array): global x, y, z, in meters.
            target_velocity ((3,) NumPy array, optional): global, in m/s.

        Returns:
            (4,) NumPy array with the desired RPMs of each propeller.
        """
        self.control_counter += 1
        ############################################################
        ############################################################
        #### HOMEWORK CODE (START) #################################
        ############################################################
        ############################################################
        if self.control_counter%(1/self.timestep) == 0:
            print(current_position)
            print(current_quaternion)
            print(current_velocity)
            print(current_angular_velocity)
            print(target_position)
            print(target_velocity)
        propellers_0_and_3_rpm = np.sqrt(self.gravity/(4*self.kf_coeff))
        propellers_1_and_2_rpm = np.sqrt(self.gravity/(4*self.kf_coeff))
        ############################################################
        ############################################################
        #### HOMEWORK CODE (END) ###################################
        ############################################################
        ############################################################
        return np.array([propellers_0_and_3_rpm, propellers_1_and_2_rpm,
                         propellers_1_and_2_rpm, propellers_0_and_3_rpm])
