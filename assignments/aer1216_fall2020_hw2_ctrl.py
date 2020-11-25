"""Control implementation for assignment 2.

The script is used the simulation in file `aer1216_fall2020_hw2_sim.py`.

Example
-------
To run the simulation, type in a terminal:

    $ python aer1216_fall2020_hw2_sim.py

Notes
-----
To-dos
    Fill appropriate values in the 3 by 3 matrix self.matrix_u2rpm.


"""
import numpy as np
from gym_pybullet_drones.envs.BaseAviary import BaseAviary

class HW2Control():
    """Control class for assignment 2."""

    ################################################################################

    def __init__(self,
                 env: BaseAviary,
                 control_type: int=0
                 ):
        """ Initialization of class HW2Control.

        Parameters
        ----------
        env : BaseAviary
            The PyBullet-based simulation environment.
        control_type : int, optional
            Choose between implementation of the u1 computation.

        """
        self.g = env.G
        """float: gravity acceleration, in meters per second squared."""
        self.mass = env.M
        """float: the mass of quad from environment."""
        self.inertia_xx = env.J[0][0]
        """float: the inertia of quad around x axis."""
        self.arm_length = env.L
        """float: the inertia of quad around x axis."""
        self.timestep = env.TIMESTEP
        """float: simulation and control timestep."""
        self.kf_coeff = env.KF
        """float: RPMs to force coefficient."""
        self.km_coeff = env.KM
        """float: RPMs to torque coefficient."""
        self.CTRL_TYPE = control_type
        """int: Flag switching beween implementations of u1."""
        self.p_coeff_position = {}
        """dict[str, float]: Proportional coefficient(s) for position control."""
        self.d_coeff_position = {}
        """dict[str, float]: Derivative coefficient(s) for position control."""

        ############################################################
        ############################################################
        #### HOMEWORK CODE (START) #################################
        ############################################################
        ############################################################

        # Objective 1 of 4: fill appropriate values in the 3 by 3 matrix
        self.matrix_u2rpm = np.linalg.inv(np.array([ 
                                                     [0,  0,  0],
                                                     [0,  0,  0],
                                                     [0,  0,  0] 
                                                    ]))
        """ndarray: (3, 3)-shaped array of ints to determine motor rpm from force and torque."""

        ############################################################
        ############################################################
        #### HOMEWORK CODE (END) ###################################
        ############################################################
        ############################################################

        self.p_coeff_position["z"] = 0.7 * 0.7
        self.d_coeff_position["z"] = 2 * 0.5 * 0.7
        #
        self.p_coeff_position["y"] = 0.7 * 0.7
        self.d_coeff_position["y"] = 2 * 0.5 * 0.7
        #
        self.p_coeff_position["r"] = 0.7 * 0.7
        self.d_coeff_position["r"] = 2 * 2.5 * 0.7

        self.reset()

    ################################################################################

    def reset(self):
        """ Resets the controller counter."""
        self.control_counter = 0

    ################################################################################

    def compute_control(self,
                        current_position,
                        current_velocity,
                        current_rpy,
                        current_rpy_dot,
                        target_position,
                        target_velocity=np.zeros(3),
                        target_acceleration=np.zeros(3),
                        ):
        """Computes the propellers' RPMs for the target state, given the current state.

        Parameters
        ----------
        current_position : ndarray
            (3,)-shaped array of floats containing global x, y, z, in meters.
        current_velocity : ndarray
            (3,)-shaped array of floats containing global vx, vy, vz, in m/s.
        current_rpy : ndarray
            (3,)-shaped array of floats containing roll, pitch, yaw, in rad.
        current_rpy_dot : ndarray
            (3,)-shaped array of floats containing roll_dot, pitch_dot, yaw_dot, in rad/s.
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

        ##### Calculate PD control in y, z #########################
        y_ddot = self.pd_control(target_position[1],
                                 current_position[1],
                                 target_velocity[1],
                                 current_velocity[1],
                                 target_acceleration[1],
                                 "y"
                                 )
        z_ddot = self.pd_control(target_position[2],
                                 current_position[2],
                                 target_velocity[2],
                                 current_velocity[2],
                                 target_acceleration[2],
                                 "z"
                                 )

        ##### Calculate desired roll and rates given by PD #########
        desired_roll = -y_ddot / self.g
        desired_roll_dot = (desired_roll - current_rpy[0]) / 0.004
        self.old_roll = desired_roll
        self.old_roll_dot = desired_roll_dot
        roll_ddot = self.pd_control(desired_roll, 
                                    current_rpy[0],
                                    desired_roll_dot, 
                                    current_rpy_dot[0],
                                    0,
                                    "r"
                                    )

        ############################################################
        ############################################################
        #### HOMEWORK CODE (START) #################################
        ############################################################
        ############################################################
        
        # Variable that you might use
        #   self.g
        #   self.mass
        #   self.inertia_xx
        #   y_ddot
        #   z_ddot
        #   roll_ddot
        #   current_rpy[0], current_rpy[1], current_rpy[2]
        # Basic math and NumPy
        #   sine(x) -> np.sin(x)
        #   cosine(x) -> np.cos(x)
        #   x squared -> x**2
        #   square root of x -> np.sqrt(x)

        ##### Calculate thrust and moment given the PD input #######
        if self.CTRL_TYPE == 0:
            #### Linear Control ########################################
            # Objective 2 of 4: compute u_1 for the linear controller
            u_1 = -1

        elif self.CTRL_TYPE == 1:
            #### Nonlinear Control 1 ###################################
            u_1 = self.mass * (self.g + z_ddot) / np.cos(current_rpy[0])

        elif self.CTRL_TYPE == 2:
            #### Nonlinear Control 2 ###################################
            # Objective 3 of 4: compute u_1 for the second nonlinear controller
            u_1 = -1

        # Objective 4 of 4: compute u_2
        u_2 = -1

        ############################################################
        ############################################################
        #### HOMEWORK CODE (END) ###################################
        ############################################################
        ############################################################
        
        ##### Calculate RPMs #######################################
        u = np.array([ [u_1 / self.kf_coeff],
                       [u_2 / (self.arm_length*self.kf_coeff)],
                       [0] ])
        propellers_rpm = np.dot(self.matrix_u2rpm, u)
        
        #### Command the turn rates of propellers 1 and 3 ##########
        propellers_1_rpm = np.sqrt(propellers_rpm[1, 0])
        propellers_3_rpm = np.sqrt(propellers_rpm[2, 0])

        #### For motion in the Y-Z plane, assign the same turn rates to prop. 0 and 2
        propellers_0_and_2_rpm = np.sqrt(propellers_rpm[0, 0])

        #### Print relevant output #################################
        if self.control_counter%(1/self.timestep) == 0:
            print("current_position", current_position)
            print("current_velocity", current_velocity)
            print("target_position", target_position)
            print("target_velocity", target_velocity)
            print("target_acceleration", target_acceleration)

        return np.array([propellers_0_and_2_rpm, propellers_1_rpm,
                         propellers_0_and_2_rpm, propellers_3_rpm])

    ################################################################################

    def pd_control(self,
                   desired_position,
                   current_position,
                   desired_velocity,
                   current_velocity,
                   desired_acceleration,
                   opt
                   ):
        """Computes PD control for the acceleration minimizing position error.

        Parameters
        ----------
        desired_position :
            float: Desired global position.
        current_position :
            float: Current global position.
        desired_velocity :
            float: Desired global velocity.
        current_velocity :
            float: Current global velocity.
        desired_acceleration :
            float: Desired global acceleration.

        Returns
        -------
        float
            The commanded acceleration.
        """
        u = desired_acceleration + \
            self.d_coeff_position[opt] * (desired_velocity - current_velocity) + \
            self.p_coeff_position[opt] * (desired_position - current_position)

        return u
