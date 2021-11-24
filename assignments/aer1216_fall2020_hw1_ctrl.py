"""Control implementation for assignment 1.

The controller used the simulation in file `aer1216_fall2020_hw1_sim.py`.

Example
-------
To run the simulation, type in a terminal:

    $ python aer1216_fall2020_hw1_sim.py

Notes
-----
Tune the PD coefficients in `HW1Control.__init__()`.

"""
import numpy as np
from gym_pybullet_drones.envs.BaseAviary import BaseAviary

class HW1Control():
    """Control class for assignment 1."""

    ################################################################################

    def __init__(self,
                 env: BaseAviary
                 ):
        """ Initialization of class HW1Control.

        Parameters
        ----------
        env : BaseAviary
            The PyBullet-based simulation environment.

        """
        self.g = env.G
        """float: Gravity acceleration, in meters per second squared."""
        self.mass = env.M
        """float: The mass of quad from environment."""
        self.timestep = env.TIMESTEP
        """float: Simulation and control timestep."""
        self.kf_coeff = env.KF
        """float: RPMs to force coefficient."""
        self.km_coeff = env.KM
        """float: RPMs to torque coefficient."""

        ############################################################
        ############################################################
        #### HOMEWORK CODE (START) #################################
        ############################################################
        ############################################################
        self.p_coeff_position = 0.7 * 0.7
        """float: Proportional coefficient for position control."""
        self.d_coeff_position = 2 * 0.7 * 0.7
        """float: Derivative coefficient for position control."""
        ############################################################
        ############################################################
        #### HOMEWORK CODE (END) ###################################
        ############################################################
        ############################################################

        self.reset()

    ################################################################################

    def reset(self):
        """ Resets the controller counter."""
        self.control_counter = 0

    ################################################################################

    def compute_control(self,
                        current_position,
                        current_velocity,
                        target_position,
                        target_velocity=np.zeros(3),
                        target_acceleration=np.zeros(3),
                        ):
        """Compute the propellers' RPMs for the target state, given the current state.

        Parameters
        ----------
        current_position : ndarray
            (3,)-shaped array of floats containing global x, y, z, in meters.
        current_velocity : ndarray
            (3,)-shaped array of floats containing global vx, vy, vz, in m/s.
        target_position : ndarray
            (3,)-shaped array of float containing global x, y, z, in meters.
        target_velocity : ndarray, optional
            (3,)-shaped array of floats containing global, in m/s.
        target_acceleration : ndarray, optional
            (3,)-shaped array of floats containing global, in m/s^2.

        Returns
        -------
        ndarray
            (4,)-shaped array of ints containing the desired RPMs of each propeller.
        """
        self.control_counter += 1
        
        ############################################################
        ############################################################
        #### HOMEWORK CODE (START) #################################
        ############################################################
        ############################################################

        ##### Calculate position and velocity errors ###############
        current_pos_error = target_position[2] - current_position[2]
        current_vel_error = target_velocity[2] - current_velocity[2]

        #### Calculate input with a PD controller ##################
        # u = desired_acceleration + Kv * velocity_error + Kp * position_error
        u = target_acceleration[2] \
            + self.d_coeff_position * current_vel_error \
            + self.p_coeff_position * current_pos_error

        ##### Calculate propeller turn rates given the PD input ####
        # turn_rate = sqrt( (m*u + m*g) / (4*Kf) )
        propellers_rpm = np.sqrt((u*self.mass + self.g*self.mass) / (4 * self.kf_coeff))

        # For up-down motion, assign the same turn rates to all motors
        propellers_0_and_3_rpm, propellers_1_and_2_rpm = propellers_rpm, propellers_rpm
        ############################################################
        ############################################################
        #### HOMEWORK CODE (END) ###################################
        ############################################################
        ############################################################

        #### Print relevant output #################################
        if self.control_counter%(1/self.timestep) == 0:
            print("current_position", current_position)
            print("current_velocity", current_velocity)
            print("target_position", target_position)
            print("target_velocity", target_velocity)
            print("target_acceleration", target_acceleration)

        return np.array([propellers_0_and_3_rpm, propellers_1_and_2_rpm,
                         propellers_1_and_2_rpm, propellers_0_and_3_rpm])
