"""Script demonstrating the use of crazyflie-firmware's Python bindings.

Example
-------
In a terminal, run as:

    $ python cff.py

"""
import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import cffirmware
from scipy.spatial.transform import Rotation as R

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = True
DEFAULT_SIMULATION_FREQ_HZ = 500
DEFAULT_CONTROL_FREQ_HZ = 500
DEFAULT_DURATION_SEC = 14
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False

def run(
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB
        ):
    #### Initialize the simulation #############################
    INIT_XYZS = np.array([[.5*i, .5*i, .05] for i in range(num_drones)])
    INIT_RPYS = np.array([[0, 0,  0] for i in range(num_drones)])

    #### Initialize a circular trajectory ######################
    # PERIOD = 10
    # NUM_WP = control_freq_hz*PERIOD
    # TARGET_POS = np.zeros((NUM_WP,3))
    # for i in range(NUM_WP):
    #     TARGET_POS[i, :] = R*np.cos((i/NUM_WP)*(2*np.pi)+np.pi/2)+INIT_XYZS[0, 0], R*np.sin((i/NUM_WP)*(2*np.pi)+np.pi/2)-R+INIT_XYZS[0, 1], 0
    # wp_counters = np.array([int((i*NUM_WP/6)%NUM_WP) for i in range(num_drones)])

    delta = 2*DEFAULT_CONTROL_FREQ_HZ
    trajectory = [[0, 0, 0] for i in range(delta)] + \
        [[0, 0, i/delta] for i in range(delta)] + \
        [[i/delta, 0, 1] for i in range(delta)] + \
        [[1, i/delta, 1] for i in range(delta)] + \
        [[1-i/delta, 1, 1] for i in range(delta)] + \
        [[0, 1-i/delta, 1] for i in range(delta)] + \
        [[0, 0, 1-i/delta] for i in range(delta)]

    #### Create the environment ################################
    env = CtrlAviary(drone_model=drone,
                        num_drones=num_drones,
                        initial_xyzs=INIT_XYZS,
                        initial_rpys=INIT_RPYS,
                        physics=physics,
                        neighbourhood_radius=10,
                        pyb_freq=simulation_freq_hz,
                        ctrl_freq=control_freq_hz,
                        gui=gui,
                        record=record_video,
                        obstacles=obstacles,
                        user_debug_gui=user_debug_gui
                        )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]

    cff_controller = cffirmware.controllerMellinger_t()
    cffirmware.controllerMellingerInit(cff_controller)

    #### Run the simulation ####################################
    action = np.zeros((num_drones,4))
    START = time.time()

    pid_rpm1 = []
    pid_rpm2 = []
    pid_rpm3 = []
    pid_rpm4 = []

    cff_rpm1 = []
    cff_rpm2 = []
    cff_rpm3 = []
    cff_rpm4 = []

    prev_rpy = [0, 0, 0]
    prev_vel = [0, 0, 0]


    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        #### Make it rain rubber ducks #############################
        # if i/env.SIM_FREQ>5 and i%10==0 and i/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        for j in range(num_drones):

            try:
                target = trajectory[i]
                pos = target+[INIT_XYZS[j][0], INIT_XYZS[j][1], 0]
                vel = np.zeros(3)
                acc = np.zeros(3)
                yaw = i*np.pi/delta/2
                rpy_rate = np.zeros(3)
                # print(pos)
            except:
                break

            action[j, :], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                        state=obs[j],
                                                        target_pos=[pos[0], pos[1], pos[2]],
                                                        # target_pos=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2]]),
                                                        # target_pos=INIT_XYZS[j, :] + TARGET_POS[wp_counters[j], :],
                                                        target_rpy=INIT_RPYS[j, :]
                                                        )



            state = cffirmware.state_t()

            cur_pos=np.array([obs[j][0], obs[j][1], obs[j][2]]) # global coord, m

            state.position.x = cur_pos[0]
            state.position.y = cur_pos[1]
            state.position.z = cur_pos[2]

            cur_vel=np.array([obs[j][10], obs[j][11], obs[j][12]]) # global coord, m/s

            state.velocity.x = cur_vel[0]
            state.velocity.y = cur_vel[1]
            state.velocity.z = cur_vel[2]

            cur_acc = (cur_vel - prev_vel) / env.CTRL_FREQ / 9.8 + np.array([0, 0, 1]) # global coord
            prev_vel = cur_vel

            state.acc.x = cur_acc[0]
            state.acc.y = cur_acc[1]
            state.acc.z = cur_acc[2]

            cur_rpy = np.array([obs[j][7], obs[j][8], obs[j][9]]) # body coord, rad 

            # state.attitude.roll = cur_rpy[0]
            # state.attitude.pitch = -cur_rpy[1] # WARNING: This needs to be negated
            # state.attitude.yaw = cur_rpy[2]

            qx, qy, qz, qw = _get_quaternion_from_euler(cur_rpy[0], cur_rpy[1], cur_rpy[2]) 

            state.attitudeQuaternion.x = qx
            state.attitudeQuaternion.y = qy
            state.attitudeQuaternion.z = qz
            state.attitudeQuaternion.w = qw

            # state.timestamp = int(i / env.CTRL_FREQ * 1e3)







            sensors = cffirmware.sensorData_t()

            body_rot = R.from_euler('XYZ', cur_rpy).inv()

            cur_rotation_rates = (cur_rpy - prev_rpy) / env.CTRL_FREQ # body coord, rad/s
            prev_rpy = cur_rpy

            s_gy = cur_rotation_rates * 180/math.pi
            s_acc = body_rot.apply(cur_acc)

            sensors.gyro.x = s_gy[0]
            sensors.gyro.y = s_gy[1]
            sensors.gyro.z = s_gy[2]
            sensors.acc.x = s_acc[0]
            sensors.acc.y = s_acc[1]
            sensors.acc.z = s_acc[2]



            setpoint = cffirmware.setpoint_t()

            setpoint.position.x = pos[0]
            setpoint.position.y = pos[1]
            setpoint.position.z = pos[2]
            setpoint.velocity.x = vel[0]
            setpoint.velocity.y = vel[1]
            setpoint.velocity.z = vel[2]
            setpoint.acceleration.x = acc[0]
            setpoint.acceleration.y = acc[1]
            setpoint.acceleration.z = acc[2]

            setpoint.attitudeRate.roll = rpy_rate[0] * 180/np.pi
            setpoint.attitudeRate.pitch = rpy_rate[1] * 180/np.pi
            setpoint.attitudeRate.yaw = rpy_rate[2] * 180/np.pi

            # setpoint.timestamp = int(i/env.CTRL_FREQ*1000) # TODO: This may end up skipping control loops 

            quat = _get_quaternion_from_euler(0, 0, yaw)
            setpoint.attitudeQuaternion.x = quat[0]
            setpoint.attitudeQuaternion.y = quat[1]
            setpoint.attitudeQuaternion.z = quat[2]
            setpoint.attitudeQuaternion.w = quat[3]

            setpoint.mode.x = cffirmware.modeAbs
            setpoint.mode.y = cffirmware.modeAbs
            setpoint.mode.z = cffirmware.modeAbs

            setpoint.mode.quat = cffirmware.modeAbs
            setpoint.mode.roll = cffirmware.modeDisable
            setpoint.mode.pitch = cffirmware.modeDisable
            setpoint.mode.yaw = cffirmware.modeDisable




            control_t = cffirmware.control_t()
            step = 0
            cffirmware.controllerMellinger(cff_controller, control_t, setpoint, sensors, state, step)
            assert control_t.controlMode == cffirmware.controlModeLegacy
            # print(control.thrust, control.roll, control.pitch, control.yaw)

            r = control_t.roll / 2
            p = control_t.pitch / 2
            motor_pwms = []
            motor_pwms += [_motorsGetPWM(_limitThrust(control_t.thrust - r + p + control_t.yaw))]
            motor_pwms += [_motorsGetPWM(_limitThrust(control_t.thrust - r - p - control_t.yaw))]
            motor_pwms += [_motorsGetPWM(_limitThrust(control_t.thrust + r - p + control_t.yaw))]
            motor_pwms += [_motorsGetPWM(_limitThrust(control_t.thrust + r + p - control_t.yaw))]

            actual = cffirmware.motors_thrust_uncapped_t()
            cffirmware.powerDistribution(control_t, actual)
            # actual = cffirmware.motors_thrust_pwm_t()
            # isCapped = cffirmware.powerDistributionCap(input, actual)
            # print(actual.motors.m1, actual.motors.m2, actual.motors.m3, actual.motors.m4)
            PWM2RPM_SCALE = 0.2685
            PWM2RPM_CONST = 4070.3
            MIN_PWM = 20000
            MAX_PWM = 65535
            new_action = PWM2RPM_SCALE * np.clip(np.array([actual.motors.m1, actual.motors.m2, actual.motors.m3, actual.motors.m4]), MIN_PWM, MAX_PWM) + PWM2RPM_CONST
            print(i, new_action)
            print(i, action[j, :])
            print()

            # compare plot
            pid_rpm1.append(action[j, 0])
            pid_rpm2.append(action[j, 1])
            pid_rpm3.append(action[j, 2])
            pid_rpm4.append(action[j, 3])

            cff_rpm1.append(new_action[0])
            cff_rpm2.append(new_action[1])
            cff_rpm3.append(new_action[2])
            cff_rpm4.append(new_action[3])

            action[j, :] = [new_action[1], new_action[2], new_action[3], new_action[0]] 
            action[j, :] = motor_pwms

        #### Go to the next way point and loop #####################
        # for j in range(num_drones):
        #     wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP-1) else 0

        #### Log the simulation ####################################
        for j in range(num_drones):
            logger.log(drone=j,
                       timestamp=i/env.CTRL_FREQ,
                       state=obs[j],
                       # control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_XYZS[j, 2], INIT_RPYS[j, :], np.zeros(6)])
                       # control=np.hstack([INIT_XYZS[j, :]+TARGET_POS[wp_counters[j], :], INIT_RPYS[j, :], np.zeros(6)])
                       )

        #### Printout ##############################################
        # env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    fig, axs = plt.subplots(4, 2)
    axs[0, 0].plot(pid_rpm1)
    axs[1, 0].plot(pid_rpm2)
    axs[2, 0].plot(pid_rpm3)
    axs[3, 0].plot(pid_rpm4)

    axs[0, 1].plot(cff_rpm1)
    axs[1, 1].plot(cff_rpm2)
    axs[2, 1].plot(cff_rpm3)
    axs[3, 1].plot(cff_rpm4)
    plt.show()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("pid") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()

def _get_quaternion_from_euler(roll, pitch, yaw):
    """Convert an Euler angle to a quaternion.
    
    Args:
        roll (float): The roll (rotation around x-axis) angle in radians.
        pitch (float): The pitch (rotation around y-axis) angle in radians.
        yaw (float): The yaw (rotation around z-axis) angle in radians.
    
    Returns:
        list: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]

MAX_PWM = 65535
SUPPLY_VOLTAGE = 3
def _motorsGetPWM(thrust):
    thrust = thrust / 65536 * 60
    volts = -0.0006239 * thrust**2 + 0.088 * thrust
    percentage = min(1, volts / SUPPLY_VOLTAGE)
    ratio = percentage * MAX_PWM

    return ratio

def _limitThrust(val):
    if val > MAX_PWM:
        return MAX_PWM
    elif val < 0:
        return 0
    return val

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=DEFAULT_NUM_DRONES,          type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=DEFAULT_RECORD_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=DEFAULT_OBSTACLES,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--colab',              default=DEFAULT_COLAB, type=bool,           help='Whether example is being run by a notebook (default: "False")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
