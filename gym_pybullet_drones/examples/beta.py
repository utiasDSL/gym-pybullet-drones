"""Control + Betaflight. 

Setup
-----
Step 1: Clone and open betaflight's source:
    $ git clone https://github.com/betaflight/betaflight
    $ cd betaflight/
    $ code ./src/main/main.c

Step 2: Comment out line `delayMicroseconds_real(50); // max rate 20kHz`
    (https://github.com/betaflight/betaflight/blob/master/src/main/main.c#L52)
    from Betaflight's `SIMULATOR_BUILD` and compile:
    $ cd betaflight/
    $ make arm_sdk_install 
    $ make TARGET=SITL 

Step 3: Install betaflight configurator (https://github.com/betaflight/betaflight-configurator/releases):
    $ wget https://github.com/betaflight/betaflight-configurator/releases/download/10.9.0/betaflight-configurator_10.9.0_amd64.deb
    $ sudo dpkg -i betaflight-configurator_10.9.0_amd64.deb 
    If needed, also run:
        $ sudo apt install libgconf-2-4
        $ sudo apt --fix-broken install

Step 4: Load the configuration file onto the target using the BF configurator:
    First, start the SITL controller:
        $ ./obj/main/betaflight_SITL.elf
    Type address `tcp://localhost:5761` (top right) and click `Connect`
    Find the `Presets` tab (on the left) -> `Load backup` -> select file `../assets/beta.txt`
    Restart `betaflight_SITL.elf`

Example
-------
In one terminal run the SITL Betaflight:

    $ cd betaflight/
    $ ./obj/main/betaflight_SITL.elf

In a separate  terminal, run:

    $ cd gym-pybullet-drones/gym_pybullet_drones/examples/
    $ python beta.py

"""
import os
import time
import argparse
from datetime import datetime
import socket
import struct
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import csv

from transforms3d.quaternions import rotate_vector, qconjugate, mat2quat, qmult
from transforms3d.utils import normalized_vector

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_DRONES = DroneModel("racer")
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_SIMULATION_FREQ_HZ = 500
DEFAULT_CONTROL_FREQ_HZ = 500
DEFAULT_DURATION_SEC = 20
DEFAULT_OUTPUT_FOLDER = 'results'

UDP_IP = "127.0.0.1"

sock = socket.socket(socket.AF_INET,    # Internet
                     socket.SOCK_DGRAM) # UDP

sock_pwm = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM)  # UDP
sock_pwm.bind((UDP_IP, 9002))
sock_pwm.settimeout(0.0)

# sock_raw = socket.socket(socket.AF_INET, # Internet
#                      socket.SOCK_DGRAM)  # UDP
# sock_raw.bind((UDP_IP, 9001))
# sock_raw.settimeout(0.0)


def run(
        drone=DEFAULT_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        ):
    #### Create the environment with or without video capture ##
    env = CtrlAviary(drone_model=drone,
                        num_drones=1,
                        initial_xyzs=np.array([[.0, .0, .1]]),
                        initial_rpys=np.array([[.0, .0, .0]]),
                        physics=physics,
                        pyb_freq=simulation_freq_hz,
                        ctrl_freq=control_freq_hz,
                        gui=gui,
                        user_debug_gui=user_debug_gui
                        )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=1,
                    output_folder=output_folder,
                    )

    #### Run the simulation ####################################
    with open("../assets/beta.csv", mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        trajectory = iter([{
            "pos": np.array([
                float(row["p_x"]),
                float(row["p_y"]),
                float(row["p_z"]),
            ]),
            "vel": np.array([
                float(row["v_x"]),
                float(row["v_y"]),
                float(row["v_z"]),
            ]),
        } for row in csv_reader])
    action = np.zeros((1,4))
    ARM_TIME = 1.
    TRAJ_TIME = 1.5
    START = time.time()
    for i in range(0, int(duration_sec*env.CTRL_FREQ)):

        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### State message to Betaflight ###########################
        o = obs[0] # p, q, euler, v, w, rpm (all in world frame)
        p = o[:3]
        q = np.array([o[6], o[3], o[4], o[5]]) # w, x, y, z
        v = o[10:13]
        w = o[13:16] # world frame
        w_body = rotate_vector(w, qconjugate(q)) # local frame
        t = i/env.CTRL_FREQ
        fdm_packet = struct.pack(
            '@dddddddddddddddddd', # t, w, a, q, v, p, pressure
            t, # double timestamp in seconds
            # minus signs due to ENU to NED conversion
            w_body[0], -w_body[1], -w_body[2], # double imu_angular_velocity_rpy[3]
            0, 0, 0, # double imu_linear_acceleration_xyz[3]
            1., .0, .0, 0., # double imu_orientation_quat[4] w, x, y, z
            0, 0, 0, # double velocity_xyz[3]
            0, 0, 0, # double position_xyz[3]
            1.0 # double pressure;
        )
        sock.sendto(fdm_packet, (UDP_IP, 9003))

        #### RC defaults to Betaflight #############################
        thro = 1000 # Positive up
        yaw = 1500 # Positive CCW
        pitch = 1500 # Positive forward in x
        roll = 1500 # Positive right/forward in y
        if t > TRAJ_TIME:
            try:
                target = next(trajectory)
            except:
                break
            
            thro, roll, pitch, yaw = ctbr_controller(
                target["pos"],
                target["vel"],
                # np.array([.0, .0, z]),
                # np.array([.0, .0, vz]),
                p, v, q
            )
            thro, roll, pitch, yaw = ctbr2beta(thro, roll, pitch, yaw)

        #### RC message to Betaflight ##############################
        aux1 = 1000 if t < ARM_TIME else 1500 # Arm
        rc_packet = struct.pack(
            '@dHHHHHHHHHHHHHHHH', 
            t, # datetime.now().timestamp(), # double timestamp; // in seconds
            round(roll), round(pitch), round(thro), round(yaw),              # roll, pitch, throttle, yaw
            aux1, 1000, 1000, 1000,              # aux 1, ..
            1000, 1000, 1000, 1000, 
            1000, 1000, 1000, 1000
        )
        # print("rc", struct.unpack('@dHHHHHHHHHHHHHHHH', rc_packet))
        sock.sendto(rc_packet, (UDP_IP, 9004))

        #### PWMs message from Betaflight ##########################
        try:
            data, addr = sock_pwm.recvfrom(16) # buffer size is 100 bytes (servo_packet size 16)
        except socket.error as msg:
            # print(msg)
            pass
        else:
            # print("received message: ", data)
            betaflight_pwms = np.array(struct.unpack('@ffff', data))
            remapped_input = np.array([
                                        betaflight_pwms[2], # 0
                                        betaflight_pwms[1], # 1
                                        betaflight_pwms[3], # 2
                                        betaflight_pwms[0]  # 3
                                    ]) # Betaflight SITL motor mapping
            action =  np.array([np.sqrt(env.MAX_THRUST / 4 / env.KF * remapped_input)])

        #### Log the simulation ####################################
        logger.log(drone=0,
                    timestamp=i/env.CTRL_FREQ,
                    state=obs[0]
                    )

        #### Printout ##############################################
        env.render()

        #### Sync the simulation ###################################
        if gui:
            pass
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()
    logger.save_as_csv("beta") # Optional CSV save

    #### Plot the simulation results ###########################
    if plot:
        logger.plot()

def ctbr_controller(tar_pos, tar_vel, cur_pos, cur_vel, cur_att):
    G = np.array([.0, .0, -9.8])
    K_P = np.array([3., 3., 8.])
    K_D = np.array([2.5, 2.5, 5.])
    K_RATES = np.array([5., 5., 1.])
    P = tar_pos - cur_pos
    D = tar_vel - cur_vel
    tar_acc = K_P * P + K_D * D - G
    norm_thrust = np.dot(tar_acc, rotate_vector([.0, .0, 1.], cur_att))
    # Calculate target attitude
    z_body = normalized_vector(tar_acc)
    x_body = normalized_vector(np.cross(np.array([.0, 1., .0]), z_body))
    y_body = normalized_vector(np.cross(z_body, x_body))
    tar_att = mat2quat(np.vstack([x_body, y_body, z_body]).T)
    # Calculate body rates
    q_error = qmult(qconjugate(cur_att), tar_att)
    body_rates = 2 * K_RATES * q_error[1:]
    if q_error[0] < 0:
        body_rates = -body_rates
    return norm_thrust, *body_rates

def ctbr2beta(thrust, roll, pitch, yaw):
    MIN_CHANNEL = 1000
    MAX_CHANNEL = 2000
    MAX_RATE = 360
    MAX_THRUST = 40.9
    mid = (MAX_CHANNEL + MIN_CHANNEL) / 2
    d = (MAX_CHANNEL - MIN_CHANNEL) / 2
    thrust = thrust / MAX_THRUST * d * 2 + MIN_CHANNEL
    rates = np.array([roll, pitch, -yaw])
    rates = rates / np.pi * 180 / MAX_RATE * d + mid
    thrust = np.clip(thrust, MIN_CHANNEL, MAX_CHANNEL)
    rates = np.clip(rates, MIN_CHANNEL, MAX_CHANNEL)
    return thrust, *rates

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Test flight script using SITL Betaflight')
    parser.add_argument('--drone',              default=DEFAULT_DRONES,     type=DroneModel,    help='Drone model (default: BETA)', metavar='', choices=DroneModel)
    parser.add_argument('--physics',            default=DEFAULT_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--gui',                default=DEFAULT_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--plot',               default=DEFAULT_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=DEFAULT_USER_DEBUG_GUI,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=DEFAULT_CONTROL_FREQ_HZ,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=DEFAULT_DURATION_SEC,         type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--output_folder',     default=DEFAULT_OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    ARGS = parser.parse_args()

    run(**vars(ARGS))
