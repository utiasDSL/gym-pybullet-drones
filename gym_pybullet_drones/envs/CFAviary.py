import numpy as np
from gymnasium import spaces
import socket
import math 

from scipy.spatial.transform import Rotation as R

from gym_pybullet_drones.envs.BaseAviary import BaseAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
try:
    import pycffirmware as firm 
except ImportError:
    raise "PyCFFirmware required for CF Aviary. Please install it from https://github.com/utiasDSL/pycffirmware or use a different aviary class."


class CFAviary(BaseAviary):
    """Multi-drone environment class for use of BetaFlight controller."""
    ACTION_DELAY = 0 # how many firmware loops run between the controller commanding an action and the drone motors responding to it
    SENSOR_DELAY = 0 # how many firmware loops run between experiencing a motion and the sensors registering it
    STATE_DELAY = 0 # not yet supported, keep 0
    CONTROLLER = 'mellinger' # specifies controller type 

    # Configurations to match firmware. Not recommended to change
    GYRO_LPF_CUTOFF_FREQ = 80
    ACCEL_LPF_CUTOFF_FREQ = 30
    QUAD_FORMATION_X = True
    MOTOR_SET_ENABLE = True

    RAD_TO_DEG = 180 / math.pi

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics=Physics.PYB,
                 pyb_freq: int = 500,
                 ctrl_freq: int = 25,
                 gui=False,
                 record=False,
                 obstacles=False,
                 user_debug_gui=True,
                 output_folder='results',
                 verbose=False,
                 ):
        """Initialization of an aviary environment for use of BetaFlight controller.

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
            Whether to save a video of the simulation in folder `files/videos/`.
        obstacles : bool, optional
            Whether to add obstacles to the simulation.
        user_debug_gui : bool, optional
            Whether to draw the drones' axes and the GUI RPMs sliders.
        udp_ip : base ip for betaflight controller emulator 

        """
        firmware_freq = 500 if self.CONTROLLER == "mellinger" else 1000
        assert (pyb_freq % firmware_freq == 0), f"pyb_freq ({pyb_freq}) must be a multiple of firmware_freq ({firmware_freq}) for CFAviary."
        if num_drones != 1: 
            raise NotImplementedError("Multi-agent support for CF Aviary is not yet implemented.")
        
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         pyb_freq=pyb_freq,
                         ctrl_freq=firmware_freq, # ctrl_freq in this variable (self.CTRL_FREQ) corresponds to ctrl rate from aviary, in this case, firmware_freq
                         gui=gui,
                         record=record,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui,
                         output_folder=output_folder
                         )
        
        # Initialize connection to BetaFlight Controller 
        self.firmware_freq = firmware_freq
        self.ctrl_freq = ctrl_freq

        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535
        self.verbose = verbose

        self._initalize_cffirmware()
    
    def _initalize_cffirmware(self):
        """Resets the firmware_wrapper object.

        Todo:
            * Add support for state estimation 
        """
        self.states = []
        self.takeoff_sent = False

        # Initialize history  
        self.action_history = [[0, 0, 0, 0] for _ in range(self.ACTION_DELAY)]
        self.sensor_history = [[[0, 0, 0], [0, 0, 0]] for _ in range(self.SENSOR_DELAY)]
        self.state_history = [[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]] for _ in range(self.STATE_DELAY)]

        # Initialize gyro lpf 
        self.acclpf = [firm.lpf2pData() for _ in range(3)]
        self.gyrolpf = [firm.lpf2pData() for _ in range(3)]
        for i in range(3):
            firm.lpf2pInit(self.acclpf[i], self.firmware_freq, self.GYRO_LPF_CUTOFF_FREQ)
            firm.lpf2pInit(self.gyrolpf[i], self.firmware_freq, self.ACCEL_LPF_CUTOFF_FREQ)
        
        # Initialize state objects 
        self.control = firm.control_t()
        self.setpoint = firm.setpoint_t()
        self.sensorData = firm.sensorData_t()
        self.state = firm.state_t()
        self.tick = 0
        self.pwms = [0, 0, 0, 0]
        self.action = np.array([[0, 0, 0, 0]])
        self.command_queue = []

        self.tumble_counter = 0
        self.prev_vel = np.array([0, 0, 0])
        self.prev_rpy = np.array([0, 0, 0])
        self.prev_time_s = None
        self.last_pos_pid_call = 0
        self.last_att_pid_call = 0
        
        # Initialize state flags 
        self._error = False
        self.sensorData_set = False
        self.state_set = False
        self.full_state_cmd_override = True # When true, high level commander is not called  

        # Initialize controller
        if self.CONTROLLER == 'pid':
            firm.controllerPidInit()
            print('PID controller init test:', firm.controllerPidTest())
        elif self.CONTROLLER == 'mellinger':
            firm.controllerMellingerInit()
            assert(self.firmware_freq == 500), "Mellinger controller requires a firmware frequency of 500Hz."
            print('Mellinger controller init test:', firm.controllerMellingerTest())
        
        # Reset environment 
        init_obs, init_info = super().reset()
        init_pos=np.array([init_obs[0][0], init_obs[0][1], init_obs[0][2]]) # global coord, m
        init_vel=np.array([init_obs[0][10], init_obs[0][11], init_obs[0][12]]) # global coord, m/s
        init_rpy = np.array([init_obs[0][7], init_obs[0][8], init_obs[0][9]]) # body coord, rad 
        if self.NUM_DRONES > 1: 
            raise NotImplementedError("Firmware controller wrapper does not support multiple drones.")

        # Initilaize high level commander 
        firm.crtpCommanderHighLevelInit()
        self._update_state(0, init_pos, init_vel, np.array([0.0, 0.0, 1.0]), init_rpy * self.RAD_TO_DEG)
        self._update_initial_state(init_obs[0])
        firm.crtpCommanderHighLevelTellState(self.state)
        
        self.ctrl_dt = 1 / self.ctrl_freq
        self.firmware_dt = 1 / self.firmware_freq
        
        # Initialize visualization tools 
        self.first_motor_killed_print = True

        return init_obs, init_info

    def step(self, i):
        '''Step the firmware_wrapper class and its environment. 
        This function should be called once at the rate of ctrl_freq. Step processes and high level commands, 
        and runs the firmware loop and simulator according to the frequencies set. 

        Args: 
            i: the simulation control step index
        Todo:
            * Add support for state estimation 
        '''
        t = i / self.ctrl_freq

        self._process_command_queue(t)

        while self.tick / self.firmware_freq < t + self.ctrl_dt:
            # Step the environment and print all returned information.
            obs, reward, terminated, truncated, info = super().step(self.action)

            # Get state values from pybullet
            cur_pos=np.array([obs[0][0], obs[0][1], obs[0][2]]) # global coord, m
            cur_vel=np.array([obs[0][10], obs[0][11], obs[0][12]]) # global coord, m/s
            cur_rpy = np.array([obs[0][7], obs[0][8], obs[0][9]]) # body coord, rad 
            body_rot = R.from_euler('XYZ', cur_rpy).inv()

            if self.takeoff_sent:
                self.states += [[self.tick / self.firmware_freq, cur_pos[0], cur_pos[1], cur_pos[2]]]

            # Estimate rates 
            cur_rotation_rates = (cur_rpy - self.prev_rpy) / self.firmware_dt # body coord, rad/s
            self.prev_rpy = cur_rpy
            cur_acc = (cur_vel - self.prev_vel) / self.firmware_dt / 9.8 + np.array([0, 0, 1]) # global coord
            self.prev_vel = cur_vel
            
            # Update state 
            state_timestamp = int(self.tick / self.firmware_freq * 1e3)
            if self.STATE_DELAY:
                raise NotImplementedError("State delay is not yet implemented. Leave at 0.")
                self._update_state(state_timestamp, *self.state_history[0])
                self.state_history = self.state_history[1:] + [[cur_pos, cur_vel, cur_acc, cur_rpy * self.RAD_TO_DEG]]
            else:
                self._update_state(state_timestamp, cur_pos, cur_vel, cur_acc, cur_rpy * self.RAD_TO_DEG)#, quat=cur_quat)

            # Update sensor data 
            sensor_timestamp = int(self.tick / self.firmware_freq * 1e6)
            if self.SENSOR_DELAY:
                self._update_sensorData(sensor_timestamp, *self.sensor_history[0])
                self.sensor_history = self.sensor_history[1:] + [[body_rot.apply(cur_acc), cur_rotation_rates * self.RAD_TO_DEG]]
            else:
                self._update_sensorData(sensor_timestamp, body_rot.apply(cur_acc), cur_rotation_rates * self.RAD_TO_DEG)

            # Update setpoint 
            self._updateSetpoint(self.tick / self.firmware_freq) # setpoint looks right 

            # Step controller 
            self._step_controller()

            # Get action 
            new_action = self.PWM2RPM_SCALE * np.clip(np.array(self.pwms), self.MIN_PWM, self.MAX_PWM) + self.PWM2RPM_CONST

            if self.ACTION_DELAY:
                # Delays action commands to mimic real life hardware response delay 
                action = self.action_history[0]
                self.action_history = self.action_history[1:] + [new_action]
            else:
                action = new_action

            if self._error:
                action = np.zeros(4)
                if self.first_motor_killed_print:
                    print("Drone firmware error. Motors are killed.")
                    self.first_motor_killed_print = False

            self.action = action 

        return obs, reward, terminated, truncated, info
    
    def _update_initial_state(self, obs):
        self.prev_vel = np.array([obs[10], obs[11], obs[12]])
        self.prev_rpy = np.array([obs[7], obs[8], obs[9]])


    ##################################
    ########## Sensor Data ###########
    ##################################

    def _update_sensorData(self, timestamp, acc_vals, gyro_vals, baro_vals=[1013.25, 25]):
        '''
            Axis3f acc;               // Gs
            Axis3f gyro;              // deg/s
            Axis3f mag;               // gauss
            baro_t baro;              // C, Pa
            #ifdef LOG_SEC_IMU
                Axis3f accSec;            // Gs
                Axis3f gyroSec;           // deg/s
            #endif
            uint64_t interruptTimestamp;   // microseconds 
        '''
        ## Only gyro and acc are used in controller. Mag and baro used in state etimation (not yet supported)
        self._update_acc(*acc_vals)
        self._update_gyro(*gyro_vals)
        # self._update_gyro(self.sensorData.mag, *mag_vals)
        # self._update_baro(self.sensorData.baro, *baro_vals)

        self.sensorData.interruptTimestamp = timestamp
        self.sensorData_set = True
    
    def _update_gyro(self, x, y, z):
        self.sensorData.gyro.x = firm.lpf2pApply(self.gyrolpf[0], x)
        self.sensorData.gyro.y = firm.lpf2pApply(self.gyrolpf[1], y)
        self.sensorData.gyro.z = firm.lpf2pApply(self.gyrolpf[2], z)
        
    def _update_acc(self, x, y, z):
        self.sensorData.acc.x = firm.lpf2pApply(self.acclpf[0], x)
        self.sensorData.acc.y = firm.lpf2pApply(self.acclpf[1], y)
        self.sensorData.acc.z = firm.lpf2pApply(self.acclpf[2], z)

    def _update_baro(self, baro, pressure, temperature):
        '''
        pressure: hPa 
        temp: C
        asl = m 
        '''
        baro.pressure = pressure #* 0.01 Best guess is this is because the sensor encodes raw reading two decimal places and stores as int 
        baro.temperature = temperature
        baro.asl = (((1015.7 / baro.pressure)**0.1902630958 - 1) * (25 + 273.15)) / 0.0065
    

    ##################################
    ######### State Update ###########
    ##################################

    def _update_state(self, timestamp, pos, vel, acc, rpy, quat=None):
        '''
            attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted)
            quaternion_t attitudeQuaternion;
            point_t position;         // m
            velocity_t velocity;      // m/s
            acc_t acc;                // Gs (but acc.z without considering gravity)
        '''
        self._update_attitude_t(self.state.attitude, timestamp, *rpy) # RPY required for PID and high level commander
        if self.CONTROLLER == 'mellinger':
            self._update_attitudeQuaternion(self.state.attitudeQuaternion, timestamp, *rpy) # Quat required for Mellinger 

        self._update_3D_vec(self.state.position, timestamp, *pos)
        self._update_3D_vec(self.state.velocity, timestamp, *vel)
        self._update_3D_vec(self.state.acc, timestamp, *acc)
        self.state_set = True

    def _update_3D_vec(self, point, timestamp, x, y, z):
        point.x = x
        point.y = y
        point.z = z
        point.timestamp = timestamp

    def _update_attitudeQuaternion(self, quaternion_t, timestamp, qx, qy, qz, qw=None):
        '''Updates attitude quaternion.

        Note:
            if qw is present, input is taken as a quat. Else, as roll, pitch, and yaw in deg
        '''
        quaternion_t.timestamp = timestamp

        if qw is None: # passed roll, pitch, yaw 
            qx, qy, qz, qw = _get_quaternion_from_euler(qx/self.RAD_TO_DEG, qy/self.RAD_TO_DEG, qz/self.RAD_TO_DEG) 

        quaternion_t.x = qx
        quaternion_t.y = qy
        quaternion_t.z = qz
        quaternion_t.w = qw

    def _update_attitude_t(self, attitude_t, timestamp, roll, pitch, yaw):
        attitude_t.timestamp = timestamp
        attitude_t.roll = roll
        attitude_t.pitch = -pitch # Legacy representation in CF firmware
        attitude_t.yaw = yaw


    ##################################
    ########### Controller ###########
    ################################## 

    def _step_controller(self):
        if not (self.sensorData_set):
            print("WARNING: sensorData has not been updated since last controller call.")
        if not (self.state_set):
            print("WARNING: state has not been updated since last controller call.")
        self.sensorData_set = False
        self.state_set = False

        # Check for tumbling crazyflie 
        if self.state.acc.z < -0.5: 
            self.tumble_counter += 1
        else:
            self.tumble_counter = 0
        if self.tumble_counter >= 30:
            print('WARNING: CrazyFlie is Tumbling. Killing motors to save propellers.')
            self.pwms = [0, 0, 0, 0]
            self.tick += 1
            self._error = True
            return 

        # Determine tick based on time passed, allowing us to run pid slower than the 1000Hz it was designed for
        cur_time = self.tick / self.firmware_freq
        if (cur_time - self.last_att_pid_call > 0.002) and (cur_time - self.last_pos_pid_call > 0.01):
            _tick = 0 # Runs position and attitude controller
            self.last_pos_pid_call = cur_time
            self.last_att_pid_call = cur_time
        elif (cur_time - self.last_att_pid_call > 0.002):
            self.last_att_pid_call = cur_time
            _tick = 2 # Runs attitude controller 
        else:
            _tick = 1 # Runs neither controller 

        # Step the chosen controller 
        if self.CONTROLLER == 'pid':
            firm.controllerPid(
                self.control,
                self.setpoint,
                self.sensorData,
                self.state,
                _tick
            )
        elif self.CONTROLLER == 'mellinger':
            firm.controllerMellinger(
                self.control,
                self.setpoint,
                self.sensorData,
                self.state,
                _tick
            )

        # Get pwm values from control object 
        self._powerDistribution(self.control)
        self.tick += 1

    def _updateSetpoint(self, timestep):
        if not self.full_state_cmd_override:
            firm.crtpCommanderHighLevelTellState(self.state)
            firm.crtpCommanderHighLevelUpdateTime(timestep) # Sets commander time variable --- this is time in s from start of flight 
            firm.crtpCommanderHighLevelGetSetpoint(self.setpoint, self.state)

    def _process_command_queue(self, sim_time):
        if len(self.command_queue) > 0:
            firm.crtpCommanderHighLevelStop() # Resets planner object        
            firm.crtpCommanderHighLevelUpdateTime(sim_time) # Sets commander time variable --- this is time in s from start of flight 
            command, args = self.command_queue.pop(0)
            getattr(self, command)(*args)

    def sendFullStateCmd(self, pos, vel, acc, yaw, rpy_rate, timestep):
        """Adds a sendfullstate command to command processing queue. 
        
        Notes:
            Overrides any high level commands being processed. 

        Args:
            pos (list): [x, y, z] position of the CF (m) 
            vel (list): [x, y, z] velocity of the CF (m/s)
            acc (list): [x, y, z] acceleration of the CF (m/s^2)
            yaw (float): yaw of the CF (rad)
            rpy_rate (list): roll, pitch, yaw rates (rad/s)
            timestep (float): simulation time when command is sent (s)
        """
        self.command_queue += [['_sendFullStateCmd', [pos, vel, acc, yaw, rpy_rate, timestep]]]

    def _sendFullStateCmd(self, pos, vel, acc, yaw, rpy_rate, timestep):
        # print(f"INFO_{self.tick}: Full state command sent.")
        self.setpoint.position.x = pos[0]
        self.setpoint.position.y = pos[1]
        self.setpoint.position.z = pos[2]
        self.setpoint.velocity.x = vel[0]
        self.setpoint.velocity.y = vel[1]
        self.setpoint.velocity.z = vel[2]
        self.setpoint.acceleration.x = acc[0]
        self.setpoint.acceleration.y = acc[1]
        self.setpoint.acceleration.z = acc[2]

        self.setpoint.attitudeRate.roll = rpy_rate[0] * self.RAD_TO_DEG
        self.setpoint.attitudeRate.pitch = rpy_rate[1] * self.RAD_TO_DEG
        self.setpoint.attitudeRate.yaw = rpy_rate[2] * self.RAD_TO_DEG

        quat = _get_quaternion_from_euler(0, 0, yaw)
        self.setpoint.attitudeQuaternion.x = quat[0]
        self.setpoint.attitudeQuaternion.y = quat[1]
        self.setpoint.attitudeQuaternion.z = quat[2]
        self.setpoint.attitudeQuaternion.w = quat[3]

        # self.setpoint.attitude.yaw = yaw * 180 / math.pi
        # self.setpoint.attitude.pitch = 0
        # self.setpoint.attitude.roll = 0

        # initilize setpoint modes to match cmdFullState 
        self.setpoint.mode.x = firm.modeAbs
        self.setpoint.mode.y = firm.modeAbs
        self.setpoint.mode.z = firm.modeAbs

        self.setpoint.mode.quat = firm.modeAbs
        self.setpoint.mode.roll = firm.modeDisable
        self.setpoint.mode.pitch = firm.modeDisable
        self.setpoint.mode.yaw = firm.modeDisable

        self.setpoint.timestamp = int(timestep*1000) # TODO: This may end up skipping control loops 
        self.full_state_cmd_override = True

    def sendTakeoffCmd(self, height, duration):
        """Adds a takeoff command to command processing queue. 

        Args:
            height (float): target takeoff height (m) 
            duration: (float): length of manuever
        """
        self.command_queue += [['_sendTakeoffCmd', [height, duration]]]
    def _sendTakeoffCmd(self, height, duration):
        print(f"INFO_{self.tick}: Takeoff command sent.")
        self.takeoff_sent = True
        firm.crtpCommanderHighLevelTakeoff(height, duration)
        self.full_state_cmd_override = False

    def sendTakeoffYawCmd(self, height, duration, yaw):
        """Adds a takeoffyaw command to command processing queue. 

        Args:
            height (float): target takeoff height (m) 
            duration: (float): length of manuever
            yaw (float): target yaw (rad)
        """
        self.command_queue += [['_sendTakeoffYawCmd', [height, duration, yaw]]]
    def _sendTakeoffYawCmd(self, height, duration, yaw):
        print(f"INFO_{self.tick}: Takeoff command sent.")
        firm.crtpCommanderHighLevelTakeoffYaw(height, duration, yaw)
        self.full_state_cmd_override = False

    def sendTakeoffVelCmd(self, height, vel, relative):
        """Adds a takeoffvel command to command processing queue. 

        Args:
            height (float): target takeoff height (m) 
            vel (float): target takeoff velocity (m/s)
            relative: (bool): whether takeoff height is relative to CF's current position
        """
        self.command_queue += [['_sendTakeoffVelCmd', [height, vel, relative]]]
    def _sendTakeoffVelCmd(self, height, vel, relative):
        print(f"INFO_{self.tick}: Takeoff command sent.")
        firm.crtpCommanderHighLevelTakeoffWithVelocity(height, vel, relative)
        self.full_state_cmd_override = False

    def sendLandCmd(self, height, duration):
        """Adds a land command to command processing queue. 

        Args:
            height (float): target landing height (m) 
            duration: (float): length of manuever
        """
        self.command_queue += [['_sendLandCmd', [height, duration]]]
    def _sendLandCmd(self, height, duration):
        print(f"INFO_{self.tick}: Land command sent.")
        firm.crtpCommanderHighLevelLand(height, duration)
        self.full_state_cmd_override = False

    def sendLandYawCmd(self, height, duration, yaw):
        """Adds a landyaw command to command processing queue. 

        Args:
            height (float): target landing height (m) 
            duration: (float): length of manuever
            yaw (float): target yaw (rad)
        """
        self.command_queue += [['_sendLandYawCmd', [height, duration, yaw]]]
    def _sendLandYawCmd(self, height, duration, yaw):
        print(f"INFO_{self.tick}: Land command sent.")
        firm.crtpCommanderHighLevelLandYaw(height, duration, yaw)
        self.full_state_cmd_override = False

    def sendLandVelCmd(self, height, vel, relative):
        """Adds a landvel command to command processing queue. 

        Args:
            height (float): target landing height (m) 
            vel (float): target landing velocity (m/s)
            relative: (bool): whether landing height is relative to CF's current position
        """
        self.command_queue += [['_sendLandVelCmd', [height, vel, relative]]]
    def _sendLandVelCmd(self, height, vel, relative):
        print(f"INFO_{self.tick}: Land command sent.")
        firm.crtpCommanderHighLevelLandWithVelocity(height, vel, relative)
        self.full_state_cmd_override = False

    def sendStopCmd(self):
        """Adds a stop command to command processing queue. 
        """
        self.command_queue += [['_sendStopCmd', []]]
    def _sendStopCmd(self):
        print(f"INFO_{self.tick}: Stop command sent.")
        firm.crtpCommanderHighLevelStop()
        self.full_state_cmd_override = False
        
    def sendGotoCmd(self, pos, yaw, duration_s, relative):
        """Adds a goto command to command processing queue. 

        Args:
            pos (list): [x, y, z] target position (m)
            yaw (float): target yaw (rad)
            duration_s (float): length of manuever
            relative (bool): whether setpoint is relative to CF's current position 
        """
        self.command_queue += [['_sendGotoCmd', [pos, yaw, duration_s, relative]]]
    def _sendGotoCmd(self, pos, yaw, duration_s, relative):
        print(f"INFO_{self.tick}: Go to command sent.")
        firm.crtpCommanderHighLevelGoTo(*pos, yaw, duration_s, relative)
        self.full_state_cmd_override = False

    def notifySetpointStop(self):
        """Adds a notifySetpointStop command to command processing queue. 
        """
        self.command_queue += [['_notifySetpointStop', []]]
    def _notifySetpointStop(self):
        """Adds a notifySetpointStop command to command processing queue. 
        """
        print(f"INFO_{self.tick}: Notify setpoint stop command sent.")
        firm.crtpCommanderHighLevelTellState(self.state)
        self.full_state_cmd_override = False


    ##################################
    ###### Hardware Functions ########
    ##################################

    BRUSHED = True
    SUPPLY_VOLTAGE = 3
    def _motorsGetPWM(self, thrust):
        if (self.BRUSHED):
            thrust = thrust / 65536 * 60
            volts = -0.0006239 * thrust**2 + 0.088 * thrust
            percentage = min(1, volts / self.SUPPLY_VOLTAGE)
            ratio = percentage * self.MAX_PWM

            return ratio
        else: 
            raise NotImplementedError("Emulator does not support the brushless motor configuration at this time.")

    def _limitThrust(self, val):
        if val > self.MAX_PWM:
            return self.MAX_PWM
        elif val < 0:
            return 0
        return val

    def _powerDistribution(self, control_t):
        motor_pwms = []
        if self.QUAD_FORMATION_X:
            r = control_t.roll / 2
            p = control_t.pitch / 2

            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust - r + p + control_t.yaw))]
            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust - r - p - control_t.yaw))]
            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust + r - p + control_t.yaw))]
            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust + r + p - control_t.yaw))]
        else:
            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust + control_t.pitch + control_t.yaw))]
            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust - control_t.roll - control_t.yaw))]
            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust - control_t.pitch + control_t.yaw))]
            motor_pwms += [self._motorsGetPWM(self._limitThrust(control_t.thrust + control_t.roll - control_t.yaw))]
        
        if self.MOTOR_SET_ENABLE:
            self.pwms = motor_pwms
        else:
            self.pwms = np.clip(motor_pwms, self.MIN_PWM).tolist()
    

    ##################################
    ##### Base Aviary Overrides ######
    ##################################

    def _actionSpace(self):
        """Returns the action space of the environment.

        Returns
        -------
        spaces.Box
            An ndarray of shape (NUM_DRONES, 4) for the commanded RPMs.

        """
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([[0.,           0.,           0.,           0.] for i in range(self.NUM_DRONES)])
        act_upper_bound = np.array([[self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM] for i in range(self.NUM_DRONES)])
        return spaces.Box(low=act_lower_bound, high=act_upper_bound, dtype=np.float32)
    
    ################################################################################

    def _observationSpace(self):
        """Returns the observation space of the environment.

        Returns
        -------
        spaces.Box
            The observation space, i.e., an ndarray of shape (NUM_DRONES, 20).

        """
        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ       P0            P1            P2            P3
        obs_lower_bound = np.array([[-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.,           0.,           0.,           0.] for i in range(self.NUM_DRONES)])
        obs_upper_bound = np.array([[np.inf,  np.inf,  np.inf, 1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM] for i in range(self.NUM_DRONES)])
        return spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32)

    ################################################################################

    def _computeObs(self):
        """Returns the current observation of the environment.

        For the value of the state, see the implementation of `_getDroneStateVector()`.

        Returns
        -------
        ndarray
            An ndarray of shape (NUM_DRONES, 20) with the state of each drone.

        """
        return np.array([self._getDroneStateVector(i) for i in range(self.NUM_DRONES)])

    ################################################################################

    def _preprocessAction(self,
                          action
                          ):
        """Pre-processes the action passed to `.step()` into motors' RPMs.

        Clips and converts a dictionary into a 2D array.

        Parameters
        ----------
        action : ndarray
            The (unbounded) input action for each drone, to be translated into feasible RPMs.

        Returns
        -------
        ndarray
            (NUM_DRONES, 4)-shaped array of ints containing to clipped RPMs
            commanded to the 4 motors of each drone.

        """

        return action.reshape((1,4))

    ################################################################################

    def _computeReward(self):
        """Computes the current reward value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        int
            Dummy value.

        """
        return -1

    ################################################################################
    
    def _computeTerminated(self):
        """Computes the current terminated value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        bool
            Dummy value.

        """
        return False
    
    ################################################################################
    
    def _computeTruncated(self):
        """Computes the current truncated value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        bool
            Dummy value.

        """
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