import time
import argparse
import numpy as np
from scipy.optimize import nnls

####################################################################################################
#### Sync the stepped simulation with the wall-clock ###############################################
####################################################################################################
#### Arguments #####################################################################################
#### - i (int)                              current simulation iteration ###########################
#### - start_time (timestamp)               timestamp of the simulation start ######################
#### - timestep (float)                     desired, wall-clock step of the simulation's rendering #
####################################################################################################
def sync(i, start_time, timestep):
    if timestep > .04 or i%(int(1/(24*timestep))) == 0:
        elapsed = time.time() - start_time
        if elapsed < (i*timestep):
            time.sleep(timestep*i - elapsed)

####################################################################################################
#### Convert a string into a boolean ###############################################################
####################################################################################################
#### Arguments #####################################################################################
#### - val (?)                              input value (possibly stirng) to interpret as boolean ##
####################################################################################################
#### Returns #######################################################################################
#### - _ (bool)                             the boolean interpretation of the input value ##########
####################################################################################################
def str2bool(val):
    if isinstance(val, bool):
        return val
    elif val.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif val.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError("[ERROR] in str2bool(), a Boolean value is expected")

####################################################################################################
#### Non-negative Least Squares (NNLS) RPM from desired thrust and torques  ########################
####################################################################################################
#### Arguments #####################################################################################
#### - thrust (float)                   desired thrust along the local z-axis ######################
#### - x_torque (float)                 desired x-axis torque ######################################
#### - y_torque (float)                 desired y-axis torque ######################################
#### - z_torque (float)                 desired z-axis torque ######################################
#### - ...
#### - ...
####################################################################################################
#### Returns #######################################################################################
#### - rpm ((4,1) array)                RPM values to apply to the 4 motors ########################
####################################################################################################
def nnlsRPM(thrust,
            x_torque,
            y_torque,
            z_torque,
            counter,
            max_thrust,
            max_xy_torque,
            max_z_torque,
            a,
            inv_a,
            b_coeff,
            gui=False
            ):
    #### Check the feasibility of thrust and torques ###########
    if gui and thrust < 0 or thrust > max_thrust:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible thrust {:.2f} outside range [0, {:.2f}]".format(thrust, max_thrust))
    if gui and np.abs(x_torque) > max_xy_torque:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible roll torque {:.2f} outside range [{:.2f}, {:.2f}]".format(x_torque, -max_xy_torque, max_xy_torque))
    if gui and np.abs(y_torque) > max_xy_torque:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible pitch torque {:.2f} outside range [{:.2f}, {:.2f}]".format(y_torque, -max_xy_torque, max_xy_torque))
    if gui and np.abs(z_torque) > max_z_torque:
        print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible yaw torque {:.2f} outside range [{:.2f}, {:.2f}]".format(z_torque, -max_z_torque, max_z_torque))
    B = np.multiply(np.array([thrust, x_torque, y_torque, z_torque]), b_coeff)
    sq_rpm = np.dot(inv_a, B)
    #### NNLS if any of the desired ang vel is negative ########
    if np.min(sq_rpm) < 0:
        sol, res = nnls(a,
                        B,
                        maxiter=3*a.shape[1]
                        )
        if gui:
            print("[WARNING] iter", counter, "in utils.nnlsRPM(), unfeasible squared rotor speeds, using NNLS")
            print("Negative sq. rotor speeds:\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0], sq_rpm[1], sq_rpm[2], sq_rpm[3]),
                   "\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sq_rpm[0]/np.linalg.norm(sq_rpm), sq_rpm[1]/np.linalg.norm(sq_rpm), sq_rpm[2]/np.linalg.norm(sq_rpm), sq_rpm[3]/np.linalg.norm(sq_rpm)))
            print("NNLS:\t\t\t\t [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0], sol[1], sol[2], sol[3]),
                  "\t\t\tNormalized: [{:.2f}, {:.2f}, {:.2f}, {:.2f}]".format(sol[0]/np.linalg.norm(sol), sol[1]/np.linalg.norm(sol), sol[2]/np.linalg.norm(sol), sol[3]/np.linalg.norm(sol)),
                  "\t\tResidual: {:.2f}".format(res))
        sq_rpm = sol
    return np.sqrt(sq_rpm)
