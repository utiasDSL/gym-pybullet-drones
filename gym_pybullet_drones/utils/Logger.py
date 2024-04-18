import os
from datetime import datetime

import numpy
from cycler import cycler
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from scipy.optimize import minimize
import autograd.numpy as np
from autograd import grad
from IPython.display import HTML, display

from gym_pybullet_drones.examples.gradient_descent import LossFunction
from gym_pybullet_drones.utils.plots_generation import PlotGeneration

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'


class Logger(object):
    """A class for logging and visualization.

    Stores, saves to file, and plots the kinematic information and RPMs
    of a simulation with one or more drones.

    """

    ################################################################################

    def __init__(self,
                 logging_freq_hz: int,
                 output_folder: str = "results",
                 num_drones: int = 1,
                 duration_sec: int = 0,
                 colab: bool = False
                 ):
        """Logger class __init__ method.

        Note: the order in which information is stored by Logger.log() is not the same
        as the one in, e.g., the obs["id"]["state"], check the implementation below.

        Parameters
        ----------
        logging_freq_hz : int
            Logging frequency in Hz.
        num_drones : int, optional
            Number of drones.
        duration_sec : int, optional
            Used to preallocate the log arrays (improves performance).

        """
        self.COLAB = colab
        self.OUTPUT_FOLDER = output_folder
        if not os.path.exists(self.OUTPUT_FOLDER):
            os.mkdir(self.OUTPUT_FOLDER)
        self.LOGGING_FREQ_HZ = logging_freq_hz
        self.NUM_DRONES = num_drones
        self.PREALLOCATED_ARRAYS = False if duration_sec == 0 else True
        self.counters = np.zeros(num_drones)
        self.timestamps = np.zeros((num_drones, duration_sec * self.LOGGING_FREQ_HZ))
        #### Note: this is the suggest information to log ##############################
        self.states = np.zeros((num_drones, 16, duration_sec * self.LOGGING_FREQ_HZ))  #### 16 states: pos_x,
        # pos_y,
        # pos_z,
        # vel_x,
        # vel_y,
        # vel_z,
        # roll,
        # pitch,
        # yaw,
        # ang_vel_x,
        # ang_vel_y,
        # ang_vel_z,
        # rpm0,
        # rpm1,
        # rpm2,
        # rpm3
        #### Note: this is the suggest information to log ##############################
        self.controls = np.zeros((num_drones, 12, duration_sec * self.LOGGING_FREQ_HZ))  #### 12 control targets: pos_x,
        # pos_y,
        # pos_z,
        # vel_x,
        # vel_y,
        # vel_z,
        # roll,
        # pitch,
        # yaw,
        # ang_vel_x,
        # ang_vel_y,

    ################################################################################

    def log(self,
            drone: int,
            timestamp,
            state,
            control=np.zeros(12)
            ):
        """Logs entries for a single simulation step, of a single drone.

        Parameters
        ----------
        drone : int
            Id of the drone associated to the log entry.
        timestamp : float
            Timestamp of the log in simulation clock.
        state : ndarray
            (20,)-shaped array of floats containing the drone's state.
        control : ndarray, optional
            (12,)-shaped array of floats containing the drone's control target.

        """
        if drone < 0 or drone >= self.NUM_DRONES or timestamp < 0 or len(state) != 20 or len(control) != 12:
            print("[ERROR] in Logger.log(), invalid data")
        current_counter = int(self.counters[drone])
        #### Add rows to the matrices if a counter exceeds their size
        if current_counter >= self.timestamps.shape[1]:
            self.timestamps = np.concatenate((self.timestamps, np.zeros((self.NUM_DRONES, 1))), axis=1)
            self.states = np.concatenate((self.states, np.zeros((self.NUM_DRONES, 16, 1))), axis=2)
            self.controls = np.concatenate((self.controls, np.zeros((self.NUM_DRONES, 12, 1))), axis=2)
            # self.coord = np.concatenate((self.coord, np.zeros((self.NUM_DRONES, 16, 1))), axis=2)
        #### Advance a counter is the matrices have overgrown it ###
        elif not self.PREALLOCATED_ARRAYS and self.timestamps.shape[1] > current_counter:
            current_counter = self.timestamps.shape[1] - 1
        #### Log the information and increase the counter ##########
        self.timestamps[drone, current_counter] = timestamp
        #### Re-order the kinematic obs (of most Aviaries) #########
        self.states[drone, :, current_counter] = np.hstack([state[0:3], state[10:13], state[7:10], state[13:20]])
        # self.coord[current_counter, drone, :] = np.hstack([state[0:3], state[10:13], state[7:10], state[13:20]])
        self.controls[drone, :, current_counter] = control
        self.counters[drone] = current_counter + 1

    ################################################################################

    def save(self):
        """Save the logs to file.
        """
        with open(os.path.join(self.OUTPUT_FOLDER,
                               "save-flight-" + datetime.now().strftime("%m.%d.%Y_%H.%M.%S") + ".npy"),
                  'wb') as out_file:
            np.savez(out_file, timestamps=self.timestamps, states=self.states, controls=self.controls)

    ################################################################################

    def save_as_csv(self,
                    comment: str = ""
                    ):
        """Save the logs---on your Desktop---as comma separated values.

        Parameters
        ----------
        comment : str, optional
            Added to the foldername.

        """
        csv_dir = os.path.join(self.OUTPUT_FOLDER,
                               "save-flight-" + comment + "-" + datetime.now().strftime("%m.%d.%Y_%H.%M.%S"))
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir + '/')
        t = np.arange(0, self.timestamps.shape[1] / self.LOGGING_FREQ_HZ, 1 / self.LOGGING_FREQ_HZ)
        for i in range(self.NUM_DRONES):
            with open(csv_dir + "/x" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 0, :]])), delimiter=",")
            with open(csv_dir + "/y" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 1, :]])), delimiter=",")
            with open(csv_dir + "/z" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 2, :]])), delimiter=",")
            ####
            with open(csv_dir + "/r" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 6, :]])), delimiter=",")
            with open(csv_dir + "/p" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 7, :]])), delimiter=",")
            with open(csv_dir + "/ya" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 8, :]])), delimiter=",")
            ####
            with open(csv_dir + "/rr" + str(i) + ".csv", 'wb') as out_file:
                rdot = np.hstack([0, (self.states[i, 6, 1:] - self.states[i, 6, 0:-1]) * self.LOGGING_FREQ_HZ])
                np.savetxt(out_file, np.transpose(np.vstack([t, rdot])), delimiter=",")
            with open(csv_dir + "/pr" + str(i) + ".csv", 'wb') as out_file:
                pdot = np.hstack([0, (self.states[i, 7, 1:] - self.states[i, 7, 0:-1]) * self.LOGGING_FREQ_HZ])
                np.savetxt(out_file, np.transpose(np.vstack([t, pdot])), delimiter=",")
            with open(csv_dir + "/yar" + str(i) + ".csv", 'wb') as out_file:
                ydot = np.hstack([0, (self.states[i, 8, 1:] - self.states[i, 8, 0:-1]) * self.LOGGING_FREQ_HZ])
                np.savetxt(out_file, np.transpose(np.vstack([t, ydot])), delimiter=",")
            ###
            with open(csv_dir + "/vx" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 3, :]])), delimiter=",")
            with open(csv_dir + "/vy" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 4, :]])), delimiter=",")
            with open(csv_dir + "/vz" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 5, :]])), delimiter=",")
            ####
            with open(csv_dir + "/wx" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 9, :]])), delimiter=",")
            with open(csv_dir + "/wy" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 10, :]])), delimiter=",")
            with open(csv_dir + "/wz" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 11, :]])), delimiter=",")
            ####
            with open(csv_dir + "/rpm0-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 12, :]])), delimiter=",")
            with open(csv_dir + "/rpm1-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 13, :]])), delimiter=",")
            with open(csv_dir + "/rpm2-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 14, :]])), delimiter=",")
            with open(csv_dir + "/rpm3-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, self.states[i, 15, :]])), delimiter=",")
            ####
            with open(csv_dir + "/pwm0-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 12, :] - 4070.3) / 0.2685])),
                           delimiter=",")
            with open(csv_dir + "/pwm1-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 13, :] - 4070.3) / 0.2685])),
                           delimiter=",")
            with open(csv_dir + "/pwm2-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 14, :] - 4070.3) / 0.2685])),
                           delimiter=",")
            with open(csv_dir + "/pwm3-" + str(i) + ".csv", 'wb') as out_file:
                np.savetxt(out_file, np.transpose(np.vstack([t, (self.states[i, 15, :] - 4070.3) / 0.2685])),
                           delimiter=",")

    ################################################################################

    def plot(self, pwm=False, trajs=0):
        """Logs entries for a single simulation step, of a single drone.

        Parameters
        ----------
        pwm : bool, optional
            If True, converts logged RPM into PWM values (for Crazyflies).

        """
        #### Loop over colors and line styles ######################
        # plt.rc('axes', prop_cycle=(cycler('color', ['r', 'g', 'b', 'y']) + cycler('linestyle', ['-', '--', ':', '-.'])))
        fig, axs = plt.subplots(10, 2)
        t = np.arange(0, self.timestamps.shape[1] / self.LOGGING_FREQ_HZ, 1 / self.LOGGING_FREQ_HZ)

        #### Column ################################################
        col = 0

        #### XYZ ###################################################
        row = 0
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 0, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('x (m)')

        row = 1
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 1, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('y (m)')

        row = 2
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 2, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('z (m)')

        #### RPY ###################################################
        row = 3
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 6, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('r (rad)')
        row = 4
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 7, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('p (rad)')
        row = 5
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 8, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('y (rad)')

        #### Ang Vel ###############################################
        row = 6
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 9, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('wx')
        row = 7
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 10, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('wy')
        row = 8
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 11, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('wz')

        #### Time ##################################################
        row = 9
        axs[row, col].plot(t, t, label="time")
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('time')

        #### Column ################################################
        col = 1

        #### Velocity ##############################################
        row = 0
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 3, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('vx (m/s)')
        row = 1
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 4, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('vy (m/s)')
        row = 2
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 5, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('vz (m/s)')

        #### RPY Rates #############################################
        row = 3
        for j in range(self.NUM_DRONES):
            rdot = np.hstack([0, (self.states[j, 6, 1:] - self.states[j, 6, 0:-1]) * self.LOGGING_FREQ_HZ])
            axs[row, col].plot(t, rdot, label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('rdot (rad/s)')
        row = 4
        for j in range(self.NUM_DRONES):
            pdot = np.hstack([0, (self.states[j, 7, 1:] - self.states[j, 7, 0:-1]) * self.LOGGING_FREQ_HZ])
            axs[row, col].plot(t, pdot, label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('pdot (rad/s)')
        row = 5
        for j in range(self.NUM_DRONES):
            ydot = np.hstack([0, (self.states[j, 8, 1:] - self.states[j, 8, 0:-1]) * self.LOGGING_FREQ_HZ])
            axs[row, col].plot(t, ydot, label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        axs[row, col].set_ylabel('ydot (rad/s)')

        ### This IF converts RPM into PWM for all drones ###########
        #### except drone_0 (only used in examples/compare.py) #####
        for j in range(self.NUM_DRONES):
            for i in range(12, 16):
                if pwm and j > 0:
                    self.states[j, i, :] = (self.states[j, i, :] - 4070.3) / 0.2685

        #### RPMs ##################################################
        row = 6
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 12, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM0')
        else:
            axs[row, col].set_ylabel('RPM0')
        row = 7
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 13, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM1')
        else:
            axs[row, col].set_ylabel('RPM1')
        row = 8
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 14, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM2')
        else:
            axs[row, col].set_ylabel('RPM2')
        row = 9
        for j in range(self.NUM_DRONES):
            axs[row, col].plot(t, self.states[j, 15, :], label="drone_" + str(j))
        axs[row, col].set_xlabel('time')
        if pwm:
            axs[row, col].set_ylabel('PWM3')
        else:
            axs[row, col].set_ylabel('RPM3')

        #### Drawing options #######################################
        for i in range(10):
            for j in range(2):
                axs[i, j].grid(True)
                axs[i, j].legend(loc='upper right',
                                 frameon=True
                                 )
        fig.subplots_adjust(left=0.06,
                            bottom=0.05,
                            right=0.99,
                            top=0.98,
                            wspace=0.15,
                            hspace=0.0
                            )

        if self.COLAB:
            plt.savefig(os.path.join('results', 'output_figure.png'))
        else:
            plt.show()

    def plot_trajct(self, trajs=0):

        PLOT_FS = 20
        SIMULATED_FS = 300

        trajs_s = trajs.sample(PLOT_FS)
        step = SIMULATED_FS // PLOT_FS
        uav_c = np.transpose(np.array([self.states[:, 0, :], self.states[:, 1, :], self.states[:, 2, :]]), (2, 1, 0))

        uav_coord = uav_c[::step]
        uav_coord_c = uav_coord.copy()
        usv_coord = trajs_s.xyz
        val = LossFunction.communication_quality_function(uav_coord, usv_coord)
        reward = np.sum(1 / val)
        print("Reward", reward)
        opt_x = np.zeros((usv_coord.shape[0], self.NUM_DRONES, 3))
        opt_x[0] = uav_coord_c[0, :, :]
        for i in range(1, usv_coord.shape[0]):
            function = lambda x: LossFunction.communication_quality_function(x.reshape(1, self.NUM_DRONES, 3),
                                                                             usv_coord[i, :, :].reshape(1, 4, 3))
            optimized = minimize(function, opt_x[i - 1].reshape(1, -1))
            opt_x[i] += optimized.x.reshape(self.NUM_DRONES, 3)

        opt_x[:, :, 2] += 10
        val_opt = LossFunction.communication_quality_function(opt_x, usv_coord)

        plt.rc('font', size=25)
        plt.rc('axes', titlesize=25)
        plt.rc('axes', labelsize=25)
        plt.rc('legend', fontsize=25)
        plt.rc('figure', titlesize=1000)
        fig = plt.figure(figsize=(40, 20))
        ax = fig.add_subplot(121)
        plots_usv = []
        plots_uav = []
        plots_uav_opt = []
        PlotGeneration.created_plot(plots_usv, ax, trajs.m, usv_coord, "USV")
        PlotGeneration.created_plot(plots_uav, ax, self.NUM_DRONES, uav_coord, "UAV")
        PlotGeneration.created_plot(plots_uav_opt, ax, self.NUM_DRONES, opt_x, "UAV_OPT")

        ax.set_xlabel('  x, м')
        ax.set_ylabel('  y, м')
        ax.set_title('Траектории')
        tr_min = np.min(usv_coord, axis=(0, 1))
        tr_max = np.max(usv_coord, axis=(0, 1))
        ax.set(xlim=[tr_min[0], tr_max[0]],
               ylim=[tr_min[1], tr_max[1]])
        ax.legend(fontsize=10)

        ax2 = fig.add_subplot(122)
        ax2.set(xlim=[0, trajs_s.time.n],
                ylim=[0, np.max(val)])
        ax2.set_title("Функция качества связи")
        ax2.grid()

        def update(frame):
            start_frame = max(0, frame - 100)
            start_frame_uav = max(0, frame - 10)
            PlotGeneration.update_animation(start_frame, frame, usv_coord, plots_usv)
            PlotGeneration.update_animation(start_frame_uav, frame, uav_coord, plots_uav)
            PlotGeneration.update_animation(start_frame_uav, frame, opt_x, plots_uav_opt)

            plot_val = []
            plot_opt_val = []
            plot_val += ax2.plot(val[:frame], "b")
            plot_opt_val += ax2.plot(val_opt[:frame], "r")
            full_plots = plots_usv + plots_uav + plot_val + plots_uav_opt + plot_opt_val
            return full_plots

        ani1 = animation.FuncAnimation(fig, update, frames=trajs_s.time.n, blit=True, interval=100)

        if self.COLAB:
            display(HTML(ani1.to_jshtml()))
            plt.close(fig)
            plt.savefig(os.path.join('results', 'output_figure.png'))
        else:
            ani1.save('animation4.mp4', writer='ffmpeg')
            plt.show()

        plt.close(fig)
