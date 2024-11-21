# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
# Functions get_poly_cc, minSomethingTraj, pos_waypoint_min are derived from Peter Huang's work:
# https://github.com/hbd730/quadcopter-simulation
# author: Peter Huang
# email: hbd730@gmail.com
# license: BSD
# Please feel free to use and modify this, but keep the above information. Thanks!



import numpy as np
from numpy import pi
from numpy.linalg import norm


class Trajectory:

    def __init__(self, ctrlType, trajSelect, v_avg):

        self.ctrlType = ctrlType
        self.xyzType = trajSelect[0]
        self.yawType = trajSelect[1]
        self.averVel = trajSelect[2]

        # t_wps, wps, y_wps, v_wp = makeWaypoints()
        self.v_wp  = v_avg

        self.end_reached = 0

        # Get initial heading
        self.current_heading = 0
        
        # Initialize trajectory setpoint
        self.desPos = np.zeros(3)    # Desired position (x, y, z)
        self.desVel = np.zeros(3)    # Desired velocity (xdot, ydot, zdot)
        self.desAcc = np.zeros(3)    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        self.desThr = np.zeros(3)    # Desired thrust in N-E-D directions (or E-N-U, if selected)
        self.desEul = np.zeros(3)    # Desired orientation in the world frame (phi, theta, psi)
        self.desPQR = np.zeros(3)    # Desired angular velocity in the body frame (p, q, r)
        self.desYawRate = 0.         # Desired yaw speed
        self.desJerk = np.zeros(3)   # Desired Jerk (xdotdotdot, ydotdotdot, zdotdotdot)
        self.desSnap = np.zeros(3)   # Desired Snap (xdotdotdotdot, ydotdotdotdot, zdotdotdotdot)
        self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, self.desThr, self.desEul, self.desPQR, self.desYawRate, self.desJerk, self.desSnap)).astype(float)

    def set_coefficients(self, wps):
        self.wps = np.array(wps)

        if (self.ctrlType == "xyz_pos" or self.ctrlType == "xyz_pos with geometric"):

            distance_segment = self.wps[1:] - self.wps[:-1]
            self.T_segment = np.sqrt(distance_segment[:,0]**2 + distance_segment[:,1]**2 + distance_segment[:,2]**2)/self.v_wp
            self.t_wps = np.zeros(len(self.T_segment) + 1)
            self.t_wps[1:] = np.cumsum(self.T_segment)
            
            if (self.xyzType >= 3 and self.xyzType <= 6):
                self.deriv_order = int(self.xyzType-2)       # Looking to minimize which derivative order (eg: Minimum velocity -> first order)

                # Calculate coefficients
                self.coeff_x = minSomethingTraj(self.wps[:,0], self.T_segment, self.deriv_order)
                self.coeff_y = minSomethingTraj(self.wps[:,1], self.T_segment, self.deriv_order)
                self.coeff_z = minSomethingTraj(self.wps[:,2], self.T_segment, self.deriv_order)

            elif (self.xyzType >= 7 and self.xyzType <= 9):
                self.deriv_order = int(self.xyzType-5)       # Looking to minimize which derivative order (eg: Minimum accel -> second order)

                # Calculate coefficients
                self.coeff_x = minSomethingTraj_stop(self.wps[:,0], self.T_segment, self.deriv_order)
                self.coeff_y = minSomethingTraj_stop(self.wps[:,1], self.T_segment, self.deriv_order)
                self.coeff_z = minSomethingTraj_stop(self.wps[:,2], self.T_segment, self.deriv_order)
            
            elif (self.xyzType >= 10 and self.xyzType <= 11):
                self.deriv_order = int(self.xyzType-7)       # Looking to minimize which derivative order (eg: Minimum jerk -> third order)

                # Calculate coefficients
                self.coeff_x = minSomethingTraj_faststop(self.wps[:,0], self.T_segment, self.deriv_order)
                self.coeff_y = minSomethingTraj_faststop(self.wps[:,1], self.T_segment, self.deriv_order)
                self.coeff_z = minSomethingTraj_faststop(self.wps[:,2], self.T_segment, self.deriv_order)
        
        if (self.yawType == 4):
            self.y_wps = np.zeros(len(self.t_wps))

    def desiredState(self, t, Ts, pos, psi):
        
        self.desPos = np.zeros(3)    # Desired position (x, y, z)
        self.desVel = np.zeros(3)    # Desired velocity (xdot, ydot, zdot)
        self.desAcc = np.zeros(3)    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        self.desThr = np.zeros(3)    # Desired thrust in N-E-D directions (or E-N-U, if selected)
        self.desEul = np.zeros(3)    # Desired orientation in the world frame (phi, theta, psi)
        self.desPQR = np.zeros(3)    # Desired angular velocity in the body frame (p, q, r)
        self.desYawRate = 0.         # Desired yaw speed
        self.desJerk = np.zeros(3)   # Desired Jerk (xdotdotdot, ydotdotdot, zdotdotdot)
        self.desSnap = np.zeros(3)   # Desired Snap (xdotdotdotdot, ydotdotdotdot, zdotdotdotdot)
        self.current_heading = psi
        
        
        def pos_waypoint_timed():
            
            if not (len(self.t_wps) == self.wps.shape[0]):
                raise Exception("Time array and waypoint array not the same size.")
            elif (np.diff(self.t_wps) <= 0).any():
                raise Exception("Time array isn't properly ordered.")  
            
            if (t == 0):
                self.t_idx = 0
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
            
            self.desPos = self.wps[self.t_idx,:]
                            
        
        def pos_waypoint_interp():
            
            if not (len(self.t_wps) == self.wps.shape[0]):
                raise Exception("Time array and waypoint array not the same size.")
            elif (np.diff(self.t_wps) <= 0).any():
                raise Exception("Time array isn't properly ordered.") 

            if (t == 0):
                self.t_idx = 0
                self.desPos = self.wps[0,:]
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
                self.desPos = self.wps[-1,:]
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
                scale = (t - self.t_wps[self.t_idx])/self.T_segment[self.t_idx]
                self.desPos = (1 - scale) * self.wps[self.t_idx,:] + scale * self.wps[self.t_idx + 1,:]
        
        def pos_waypoint_min():
            """ The function takes known number of waypoints and time, then generates a
            minimum velocity, acceleration, jerk or snap trajectory which goes through each waypoint. 
            The output is the desired state associated with the next waypoint for the time t.
            """
            if not (len(self.t_wps) == self.wps.shape[0]):
                raise Exception("Time array and waypoint array not the same size.")
                
            nb_coeff = self.deriv_order*2

            # Hover at t=0
            if t == 0:
                self.t_idx = 0
                self.desPos = self.wps[0,:]
            # Stay hover at the last waypoint position
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
                self.desPos = self.wps[-1,:]
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
                
                # Scaled time (between 0 and duration of segment)
                scale = (t - self.t_wps[self.t_idx])
                
                # Which coefficients to use
                start = nb_coeff * self.t_idx
                end = nb_coeff * (self.t_idx + 1)
                
                # Set desired position, velocity and acceleration
                t0 = get_poly_cc(nb_coeff, 0, scale)
                self.desPos = np.array([self.coeff_x[start:end].dot(t0), self.coeff_y[start:end].dot(t0), self.coeff_z[start:end].dot(t0)])

                t1 = get_poly_cc(nb_coeff, 1, scale)
                self.desVel = np.array([self.coeff_x[start:end].dot(t1), self.coeff_y[start:end].dot(t1), self.coeff_z[start:end].dot(t1)])

                t2 = get_poly_cc(nb_coeff, 2, scale)
                self.desAcc = np.array([self.coeff_x[start:end].dot(t2), self.coeff_y[start:end].dot(t2), self.coeff_z[start:end].dot(t2)])

                t3 = get_poly_cc(nb_coeff, 3, scale)
                self.desJerk = np.array([self.coeff_x[start:end].dot(t3), self.coeff_y[start:end].dot(t3), self.coeff_z[start:end].dot(t3)])

                t4 = get_poly_cc(nb_coeff, 4, scale)
                self.desSnap = np.array([self.coeff_x[start:end].dot(t4), self.coeff_y[start:end].dot(t4), self.coeff_z[start:end].dot(t4)])
        
        def pos_waypoint_arrived():

            dist_consider_arrived = 0.2 # Distance to waypoint that is considered as "arrived"
            if (t == 0):
                self.t_idx = 0
                self.end_reached = 0
            elif not(self.end_reached):
                distance_to_next_wp = ((self.wps[self.t_idx,0]-pos[0])**2 + (self.wps[self.t_idx,1]-pos[1])**2 + (self.wps[self.t_idx,2]-pos[2])**2)**(0.5)
                if (distance_to_next_wp < dist_consider_arrived):
                    self.t_idx += 1
                    if (self.t_idx >= len(self.wps[:,0])):    # if t_idx has reached the end of planned waypoints
                        self.end_reached = 1
                        self.t_idx = -1
                    
            self.desPos = self.wps[self.t_idx,:]

        def pos_waypoint_arrived_wait():

            dist_consider_arrived = 0.2     # Distance to waypoint that is considered as "arrived"
            if (t == 0):
                self.t_idx = 0              # Index of waypoint to go to
                self.t_arrived = 0          # Time when arrived at first waypoint ([0, 0, 0])
                self.arrived = True         # Bool to confirm arrived at first waypoint
                self.end_reached = 0        # End is not reached yet
            
            # If end is not reached, calculate distance to next waypoint
            elif not(self.end_reached):     
                distance_to_next_wp = ((self.wps[self.t_idx,0]-pos[0])**2 + (self.wps[self.t_idx,1]-pos[1])**2 + (self.wps[self.t_idx,2]-pos[2])**2)**(0.5)
                
                # If waypoint distance is below a threshold, specify the arrival time and confirm arrival
                if (distance_to_next_wp < dist_consider_arrived) and not self.arrived:
                    self.t_arrived = t
                    self.arrived = True

                # If arrived for more than xx seconds, increment waypoint index (t_idx)
                # Replace 'self.t_wps[self.t_idx]' by any number to have a fixed waiting time for all waypoints
                elif self.arrived and (t-self.t_arrived > self.t_wps[self.t_idx]):   
                    self.t_idx += 1
                    self.arrived = False

                    # If t_idx has reached the end of planned waypoints
                    if (self.t_idx >= len(self.wps[:,0])):    
                        self.end_reached = 0                  # set to  1 to stop looping
                        self.t_idx = 0                        # set to -1 to stop looping  
                    
            self.desPos = self.wps[self.t_idx,:]

        def yaw_waypoint_timed():
            
            if not (len(self.t_wps) == len(self.y_wps)):
                raise Exception("Time array and waypoint array not the same size.")
            
            self.desEul[2] = self.y_wps[self.t_idx]
                    

        def yaw_waypoint_interp():

            if not (len(self.t_wps) == len(self.y_wps)):
                raise Exception("Time array and waypoint array not the same size.")

            if (t == 0) or (t >= self.t_wps[-1]):
                self.desEul[2] = self.y_wps[self.t_idx]
            else:
                scale = (t - self.t_wps[self.t_idx])/self.T_segment[self.t_idx]
                self.desEul[2] = (1 - scale)*self.y_wps[self.t_idx] + scale*self.y_wps[self.t_idx + 1]
                
                # Angle between current vector with the next heading vector
                delta_psi = self.desEul[2] - self.current_heading
                
                # Set Yaw rate
                self.desYawRate = delta_psi / Ts 

                # Prepare next iteration
                self.current_heading = self.desEul[2]
        

        def yaw_follow():

            
            # Calculate desired Yaw
            self.desEul[2] = np.arctan2(self.desPos[1]-pos[1], self.desPos[0]-pos[0])
            self.desEul[2] = np.clip(self.desEul[2], -np.pi, np.pi)        
            # Dirty hack, detect when desEul[2] switches from -pi to pi (or vice-versa) and switch manualy current_heading 
            if (np.sign(self.desEul[2]) - np.sign(self.current_heading) and abs(self.desEul[2]-self.current_heading) >= 2*pi-0.1):
                self.current_heading = self.current_heading + np.sign(self.desEul[2])*2*pi
            
            # Angle between current vector with the next heading vector
            delta_psi = self.desEul[2] - self.current_heading
            
            # Set Yaw rate
            self.desYawRate = delta_psi / Ts 

            # Prepare next iteration
            self.current_heading = self.desEul[2]


        if (self.ctrlType == "xyz_vel"):
            if (self.xyzType == 1):
                self.sDes = testVelControl(t)

        elif (self.ctrlType == "xy_vel_z_pos"):
            if (self.xyzType == 1):
                self.sDes = testVelControl(t)
        
        elif (self.ctrlType == "xyz_pos" or self.ctrlType == "xyz_pos with geometric"):
            # Hover at [0, 0, 0]
            if (self.xyzType == 0):
                pass 
            # For simple testing
            elif (self.xyzType == 99):
                self.sDes = testXYZposition(t)   
            else:    
                # List of possible position trajectories
                # ---------------------------
                # Set desired positions at every t_wps[i]
                if (self.xyzType == 1):
                    pos_waypoint_timed()
                # Interpolate position between every waypoint, to arrive at desired position every t_wps[i]
                elif (self.xyzType == 2):
                    pos_waypoint_interp()
                # Calculate a minimum velocity, acceleration, jerk or snap trajectory
                elif (self.xyzType >= 3 and self.xyzType <= 11):
                    pos_waypoint_min()
                # Go to next waypoint when arrived at waypoint
                elif (self.xyzType == 12):
                    pos_waypoint_arrived()
                # Go to next waypoint when arrived at waypoint after waiting x seconds
                elif (self.xyzType == 13):
                    pos_waypoint_arrived_wait()
                
                # List of possible yaw trajectories
                # ---------------------------
                # Set desired yaw at every t_wps[i]
                if (self.yawType == 0):
                    pass
                elif (self.yawType == 1):
                    yaw_waypoint_timed()
                # Interpolate yaw between every waypoint, to arrive at desired yaw every t_wps[i]
                elif (self.yawType == 2):
                    yaw_waypoint_interp()
                # Have the drone's heading match its desired velocity direction
                elif (self.yawType == 3):
                    yaw_follow()

                self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, self.desThr, self.desEul, self.desPQR, self.desYawRate, self.desJerk, self.desSnap)).astype(float)
        
        return self.sDes


def get_poly_cc(n, k, t):
    """ This is a helper function to get the coeffitient of coefficient for n-th
        order polynomial with k-th derivative at time t.
    """
    assert (n > 0 and k >= 0), "order and derivative must be positive."

    cc = np.ones(n)
    D  = np.linspace(n-1, 0, n)

    for i in range(n):
        for j in range(k):
            cc[i] = cc[i] * D[i]
            D[i] = D[i] - 1
            if D[i] == -1:
                D[i] = 0

    for i, c in enumerate(cc):
        cc[i] = c * np.power(t, D[i])
    return cc


def minSomethingTraj(waypoints, times, order):
    """ This function takes a list of desired waypoint i.e. [x0, x1, x2...xN] and
    time, returns a [M*N,1] coeffitients matrix for the N+1 waypoints (N segments), 
    where M is the number of coefficients per segment and is equal to (order)*2. If one 
    desires to create a minimum velocity, order = 1. Minimum snap would be order = 4. 

    1.The Problem
    Generate a full trajectory across N+1 waypoint is made of N polynomial line segment.
    Each segment is defined as a (2*order-1)-th order polynomial defined as follow:
    Minimum velocity:     Pi = ai_0 + ai1*t
    Minimum acceleration: Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3
    Minimum jerk:         Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5
    Minimum snap:         Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5 + ai6*t^6 + ai7*t^7

    Each polynomial has M unknown coefficients, thus we will have M*N unknown to
    solve in total, so we need to come up with M*N constraints.

    2.The constraints
    In general, the constraints is a set of condition which define the initial
    and final state, continuity between each piecewise function. This includes
    specifying continuity in higher derivatives of the trajectory at the
    intermediate waypoints.

    3.Matrix Design
    Since we have M*N unknown coefficients to solve, and if we are given M*N
    equations(constraints), then the problem becomes solving a linear equation.

    A * Coeff = B

    Let's look at B matrix first, B matrix is simple because it is just some constants
    on the right hand side of the equation. There are M*N constraints,
    so B matrix will be [M*N, 1].

    Coeff is the final output matrix consists of M*N elements. 
    Since B matrix is only one column, Coeff matrix must be [M*N, 1].

    A matrix is tricky, we then can think of A matrix as a coeffient-coeffient matrix.
    We are no longer looking at a particular polynomial Pi, but rather P1, P2...PN
    as a whole. Since now our Coeff matrix is [M*N, 1], and B is [M*N, 1], thus
    A matrix must have the form [M*N, M*N].

    A = [A10 A11 ... A1M A20 A21 ... A2M ... AN0 AN1 ... ANM
        ...
        ]

    Each element in a row represents the coefficient of coeffient aij under
    a certain constraint, where aij is the jth coeffient of Pi with i = 1...N, j = 0...(M-1).
    """

    n = len(waypoints) - 1
    nb_coeff = order*2

    # initialize A, and B matrix
    A = np.zeros([nb_coeff*n, nb_coeff*n])
    B = np.zeros(nb_coeff*n)

    # populate B matrix.
    for i in range(n):
        B[i] = waypoints[i]
        B[i + n] = waypoints[i+1]

    # Constraint 1 - Starting position for every segment
    for i in range(n):
        A[i][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, 0)

    # Constraint 2 - Ending position for every segment
    for i in range(n):
        A[i+n][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, times[i])

    # Constraint 3 - Starting position derivatives (up to order) are null
    for k in range(1, order):
        A[2*n+k-1][:nb_coeff] = get_poly_cc(nb_coeff, k, 0)

    # Constraint 4 - Ending position derivatives (up to order) are null
    for k in range(1, order):
        A[2*n+(order-1)+k-1][-nb_coeff:] = get_poly_cc(nb_coeff, k, times[i])
    
    # Constraint 5 - All derivatives are continuous at each waypint transition
    for i in range(n-1):
        for k in range(1, nb_coeff-1):
            A[2*n+2*(order-1) + i*2*(order-1)+k-1][i*nb_coeff : (i*nb_coeff+nb_coeff*2)] = np.concatenate((get_poly_cc(nb_coeff, k, times[i]), -get_poly_cc(nb_coeff, k, 0)))
    
    # solve for the coefficients
    Coeff = np.linalg.solve(A, B)
    return Coeff


# Minimum acceleration/jerk/snap Trajectory, but with null velocity, accel and jerk at each waypoint
def minSomethingTraj_stop(waypoints, times, order):
    """ This function takes a list of desired waypoint i.e. [x0, x1, x2...xN] and
    time, returns a [M*N,1] coeffitients matrix for the N+1 waypoints (N segments), 
    where M is the number of coefficients per segment and is equal to (order)*2. If one 
    desires to create a minimum acceleration, order = 2. Minimum snap would be order = 4. 

    1.The Problem
    Generate a full trajectory across N+1 waypoint is made of N polynomial line segment.
    Each segment is defined as a (2*order-1)-th order polynomial defined as follow:
    Minimum velocity:     Pi = ai_0 + ai1*t
    Minimum acceleration: Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3
    Minimum jerk:         Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5
    Minimum snap:         Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5 + ai6*t^6 + ai7*t^7

    Each polynomial has M unknown coefficients, thus we will have M*N unknown to
    solve in total, so we need to come up with M*N constraints.

    Unlike the function minSomethingTraj, where continuous equations for velocity, jerk and snap are generated, 
    this function generates trajectories with null velocities, accelerations and jerks at each waypoints. 
    This will make the drone stop for an instant at each waypoint.
    """

    n = len(waypoints) - 1
    nb_coeff = order*2

    # initialize A, and B matrix
    A = np.zeros([nb_coeff*n, nb_coeff*n])
    B = np.zeros(nb_coeff*n)

    # populate B matrix.
    for i in range(n):
        B[i] = waypoints[i]
        B[i + n] = waypoints[i+1]

    # Constraint 1 - Starting position for every segment
    for i in range(n):
        A[i][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, 0)

    # Constraint 2 - Ending position for every segment
    for i in range(n):
        A[i+n][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, times[i])

    # Constraint 3 - Starting position derivatives (up to order) for each segment are null
    for i in range(n):
        for k in range(1, order):
            A[2*n + k-1 + i*(order-1)][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, k, 0)

    # Constraint 4 - Ending position derivatives (up to order) for each segment are null
    for i in range(n):
        for k in range(1, order):
            A[2*n+(order-1)*n + k-1 + i*(order-1)][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, k, times[i])
    
    # solve for the coefficients
    Coeff = np.linalg.solve(A, B)
    return Coeff

# Minimum acceleration/jerk/snap Trajectory, but with null velocity only at each waypoint
def minSomethingTraj_faststop(waypoints, times, order):
    """ This function takes a list of desired waypoint i.e. [x0, x1, x2...xN] and
    time, returns a [M*N,1] coeffitients matrix for the N+1 waypoints (N segments), 
    where M is the number of coefficients per segment and is equal to (order)*2. If one 
    desires to create a minimum acceleration, order = 2. Minimum snap would be order = 4. 

    1.The Problem
    Generate a full trajectory across N+1 waypoint is made of N polynomial line segment.
    Each segment is defined as a (2*order-1)-th order polynomial defined as follow:
    Minimum velocity:     Pi = ai_0 + ai1*t
    Minimum acceleration: Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3
    Minimum jerk:         Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5
    Minimum snap:         Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5 + ai6*t^6 + ai7*t^7

    Each polynomial has M unknown coefficients, thus we will have M*N unknown to
    solve in total, so we need to come up with M*N constraints.

    Unlike the function minSomethingTraj, where continuous equations for velocity, jerk and snap are generated, 
    and unlike the function minSomethingTraj_stop, where velocities, accelerations and jerks are equal to 0 at each waypoint,
    this function generates trajectories with only null velocities. Accelerations and above derivatives are continuous. 
    This will make the drone stop for an instant at each waypoint, and then leave in the same direction it came from.
    """

    n = len(waypoints) - 1
    nb_coeff = order*2

    # initialize A, and B matrix
    A = np.zeros([nb_coeff*n, nb_coeff*n])
    B = np.zeros(nb_coeff*n)

    # populate B matrix.
    for i in range(n):
        B[i] = waypoints[i]
        B[i + n] = waypoints[i+1]

    # Constraint 1 - Starting position for every segment
    for i in range(n):
        # print(i)
        A[i][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, 0)

    # Constraint 2 - Ending position for every segment
    for i in range(n):
        # print(i+n)
        A[i+n][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, times[i])

    # Constraint 3 - Starting velocity for every segment is null
    for i in range(n):
        # print(i+2*n)
        A[i+2*n][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 1, 0)

    # Constraint 4 - Ending velocity for every segment is null
    for i in range(n):
        # print(i+3*n)
        A[i+3*n][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 1, times[i])

    # Constraint 5 - Starting position derivatives (above velocity and up to order) are null
    for k in range(2, order):
        # print(4*n + k-2)
        A[4*n+k-2][:nb_coeff] = get_poly_cc(nb_coeff, k, 0)

    # Constraint 6 - Ending position derivatives (above velocity and up to order) are null
    for k in range(2, order):
        # print(4*n+(order-2) + k-2)
        A[4*n+k-2+(order-2)][-nb_coeff:] = get_poly_cc(nb_coeff, k, times[i])

    # Constraint 7 - All derivatives above velocity are continuous at each waypint transition
    for i in range(n-1):
        for k in range(2, nb_coeff-2):
            # print(4*n+2*(order-2)+k-2+i*(nb_coeff-4))
            A[4*n+2*(order-2)+k-2+i*(nb_coeff-4)][i*nb_coeff : (i*nb_coeff+nb_coeff*2)] = np.concatenate((get_poly_cc(nb_coeff, k, times[i]), -get_poly_cc(nb_coeff, k, 0)))
            

    # solve for the coefficients
    Coeff = np.linalg.solve(A, B)
    return Coeff



## Testing scripts

def testXYZposition(t):
    desPos = np.array([0., 0., 0.])
    desVel = np.array([0., 0., 0.])
    desAcc = np.array([0., 0., 0.])
    desThr = np.array([0., 0., 0.])
    desEul = np.array([0., 0., 0.])
    desPQR = np.array([0., 0., 0.])
    desYawRate = 30.0*pi/180
    
    if t >= 1 and t < 4:
        desPos = np.array([2, 2, 1])
    elif t >= 4:
        desPos = np.array([2, -2, -2])
        desEul = np.array([0, 0, pi/3])
    
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)

    return sDes


def testVelControl(t):
    desPos = np.array([0., 0., 0.])
    desVel = np.array([0., 0., 0.])
    desAcc = np.array([0., 0., 0.])
    desThr = np.array([0., 0., 0.])
    desEul = np.array([0., 0., 0.])
    desPQR = np.array([0., 0., 0.])
    desYawRate = 0.

    if t >= 1 and t < 4:
        desVel = np.array([3, 2, 0])
    elif t >= 4:
        desVel = np.array([3, -1, 0])
     
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)
    
    return sDes
