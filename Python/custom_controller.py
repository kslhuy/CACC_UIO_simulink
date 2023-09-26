# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains PID controllers to perform lateral and longitudinal control. """

from collections import deque
import math
import random
import time
import numpy as np
import carla
import cvxpy as cp

import scipy.signal as signal


from agents.tools.misc import get_speed , get_accel_x , compute_distance , sign
from agents.tools.data_logger import DataLoggerPlotter


class VehiclePIDController():
    """
    VehiclePIDController is the combination of two PID controllers
    (lateral and longitudinal) to perform the
    low level control a vehicle from client side
    """


    def __init__(self, vehicle,vehicle_pre,world_sensor ,args_lateral, args_longitudinal, offset=0, max_throttle=0.7, max_brake=0.3,
                 max_steering=0.8):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param args_lateral: dictionary of arguments to set the lateral PID controller
        using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param args_longitudinal: dictionary of arguments to set the longitudinal
        PID controller using the following semantics:
            K_P -- Proportional term
            K_D -- Differential term
            K_I -- Integral term
        :param offset: If different than zero, the vehicle will drive displaced from the center line.
        Positive values imply a right offset while negative ones mean a left one. Numbers high enough
        to cause the vehicle to drive through other lanes might break the controller.
        """

        self.max_brake = max_brake
        self.max_throt = max_throttle
        self.max_steer = max_steering

        self._vehicle = vehicle
        self._vehicle_pre = vehicle_pre

        
        self.past_steering = self._vehicle.get_control().steer

        self._lon_controller_PID = PIDLongitudinalController(self._vehicle, **args_longitudinal)

        self._lon_controller_CACC = CACCLongitudinalController(self._vehicle,self._vehicle_pre,world_sensor)
        
        self._lat_controller = PIDLateralController(self._vehicle, offset, **args_lateral)

    def run_step(self, target_speed, waypoint):
        """
        Execute one step of control invoking both lateral and longitudinal
        PID controllers to reach a target waypoint
        at a given target_speed.

            :param target_speed: desired vehicle speed
            :param waypoint: target location encoded as a waypoint
            :return: distance (in meters) to the waypoint
        """

        # acceleration = self._lon_controller_PID.run_step(target_speed)
        acceleration = self._lon_controller_CACC.run_step(target_speed)

        # print("cacc" + str(self._lon_controller_CACC.run_step(target_speed)))
        current_steering = self._lat_controller.run_step(waypoint)
        control = carla.VehicleControl()
        if acceleration >= 0.0:
            control.throttle = min(acceleration, 1)
            control.brake = 0.0
        else:
            control.throttle = 0.0
            control.brake = min(abs(acceleration), 1)

        # Steering regulation: changes cannot happen abruptly, can't steer too much.

        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control.steer = steering
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering

        return control


    def change_longitudinal_PID(self, args_longitudinal):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lon_controller_PID.change_parameters(**args_longitudinal)

    def change_lateral_PID(self, args_lateral):
        """Changes the parameters of the PIDLongitudinalController"""
        self._lat_controller.change_parameters(**args_lateral)


class PIDLongitudinalController():
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_speed, debug=False):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in Km/h
            :param debug: boolean for debugging
            :return: throttle control
        """
        current_speed = get_speed(self._vehicle)

        if debug:
            print('Current speed = {}'.format(current_speed))

        return self._pid_control(target_speed, current_speed)
    

    def _pid_control(self, target_speed, current_speed):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

            :param target_speed:  target speed in Km/h
            :param current_speed: current speed of the vehicle in Km/h
            :return: throttle/brake control
        """

        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        print('pid = {}'.format((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie)))
        return np.clip((self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt


class CACCLongitudinalController():
    """
    CACCLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle,vehicle_pre,world_sensor, K_1 = -0.9, K_2 = 3, h = 0.5,L_des = 8 ,l = 5, tau = 0.4, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """

        self.log = DataLoggerPlotter('data.txt' , 'control.txt')
        self._vehicle = vehicle
        self._vehicle_pre = vehicle_pre
        self._world_sensor = world_sensor
        self._K_1 = K_1
        self._K_2 = K_2
        self._K_3 = (1-h*K_1*K_2)

        self.time_now = time.time()


        self._L_des = L_des  # meter
        # self._l = (self._vehicle.bounding_box.extent.x + self._vehicle_pre.bounding_box.extent.x)
        self._l = 5


        #region MATRIX for CACC 


        self._tau = tau
        self._h = h
        self._dt = 0.04
        self.frame = 0

        # Fillter EMA
        self.alpha = 0.7


        k3 = (1-self._h*self._K_1*self._K_2)
        Ac = np.array([ [0, 1, 0],
                        [0, 0, 1],
                        [-self._K_2 / (self._tau * self._h), - k3 / (self._tau * self._h), (self._K_1 - k3) / self._tau]])
        
        Bc = np.array([[0], 
                       [0], 
                       [self._K_2 / self._tau]])
        
        Fc = np.array([[0], 
                       [0], 
                       [k3 / self._tau]])
        
        Wc = np.array([[0], 
                       [0], 
                       [(k3 - self._K_1) / self._tau]])
        
        deltac = np.array([[0], 
                           [0], 
                           [self._K_2 * (self._L_des - self._l) / (self._tau * self._h)]])
        
        Cc = np.array([[1, 0, 0], 
                       [0, 1, 0]])
        
        # Discretization

        self.Ad = np.eye(3) + self._dt * Ac
        self.Bd = self._dt * Bc
        self.Fd = self._dt * Fc
        self.Wd = self._dt * Wc
        deltad = self._dt * deltac
        self.Cd = Cc
        # Descriptor form
        self.E = np.concatenate((np.identity(np.shape(self.Ad)[0]), self.Wd), axis = 1)
        self.Ae = np.concatenate((self.Ad, np.zeros([3,1])),axis=1)
        self.Be = self.Bd
        self.Fe = self.Fd
        self.deltae = deltad
        self.Cbar = np.matmul(self.Cd,self.Ad)
        self.Ce = np.concatenate((self.Cbar , np.zeros([2,1])),axis=1)
        
        self.w = np.array([[0],
                           [0],
                           [0],
                           [0]])
        
        # self.LMI()
        
        self.M = np.matrix('0;\
                            0;\
                            0;\
                            0.8929')
        
        self.G = np.matrix('0;\
                            0;\
                            0;\
                            0.7143')
        

        # # ---------------- 

        # self.Qz = np.matrix('0.5	0;\
        #                     0.025	0;\
        #                     -0.4994	20.00000;\
        #                     1.42678794364188	-57.1429')

        # self.Pz = np.matrix('0.500624219725343	-0.0249687890137328	0;\
        #                     -0.0249687890137328	0.998751560549313	0;\
        #                     0.5	-19.9750312109863	0;\
        #                     -1.4268	57.0715177456750	2.85714285714312')

        # self.N = np.matrix('0.428219928735514	0.0304116157349804	0.000450030964910238	0;\
        #                     -0.0306997012391360	0.696502946129850	0.0349018965595880	0;\
        #                     0.552047612358187	-13.4682336686113	-0.674791802461460	0;\
        #                     -5.78489762606556	36.2772054632422	3.75242966008459	0')
        
        # self.L = np.matrix('0.286781558683888	-0.0249687890137328;\
        #                     -0.00963810175323705	0.998751560549266;\
        #                     0.223696573104526	-19.9750312109862;\
        #                     -1.28451696932724	94.2143748885355')
        


        self.N = np.matrix('0	    0	    0	    0;\
                            0	    0	    0	    0;\
                            0	    0	    0	    0;\
                            0.005	-0.1	2.6	    0')
        
        self.L = np.matrix('0.5	    -0.02;\
                            -0.02	1;\
                            0.5	    -25;\
                            -4.88	153.5')

        self.Pz = np.matrix('0.5	-0.02	0;\
                            -0.02	1	    0;\
                            0.5	    -25	    0;\
                            -1.78	89.00	3.5')

        self.Qz = np.matrix('0.5	0;\
                            0.02	0;\
                            -0.5	25;\
                            1.784	-89.28')


        # self.M = np.matmul(self.Pz,Be)
        # self.G = np.matmul(self.Pz,Fe)

        # print(f"M : {self.M}")
        # print(f"G : {self.G}")


        #endregion



        self._error_buffer = deque(maxlen=10)
        self._acceleration_buffer = deque(maxlen=2)
        self._velocity_buffer = deque(maxlen=2)

        self.cyber_atk_buffer = deque(maxlen=2)
        self.usyn_buffer = deque(maxlen=2)


        self._acceleration_pre_buffer = deque(maxlen=3)
        self._acceleration_ego_buffer = deque(maxlen=3)


        
        self.y = np.array([[0.0],
                           [0.0]])
        
        self.ksi_hat = np.array([0.0 , 0.0, 0.0, 0.0])
        self.psi = np.array([[0.0], 
                            [0.0], 
                            [0.0]])


        cutoff_freq = 2.5  # Cutoff frequency in Hz
        filter_order = 3  # Filter order
        filter_type = 'lowpass'  # Type of filter
        nyquist_freq = 0.5 * 25
        self.b, self.a = signal.butter(filter_order, cutoff_freq / nyquist_freq, btype=filter_type)



        cutoff_freq = 8  # Cutoff frequency in Hz
        filter_order = 2  # Filter order
        filter_type = 'lowpass'  # Type of filter
        nyquist_freq = 0.5 * 25
        self.b_2, self.a_2 = signal.butter(filter_order, cutoff_freq / nyquist_freq, btype=filter_type)

        # ---- PIO -----



    def run_step(self, target_speed, debug=True):
        """
        Execute one step of longitudinal control to reach a given target speed.

            :param target_speed: target speed in Km/h
            :param debug: boolean for debugging
            :return: throttle control
        """
        time_eslap = self.frame*self._dt

        """ Case 1 : no attack """
        
        # cyber_attack = 0

        """ Case 2 : with constant attack , ramp attack """
        if (time_eslap < 6 ):
            cyber_attack = 0
        elif (time_eslap >= 6 and time_eslap < 12 ):
            cyber_attack = 2*(time_eslap - 4)
        elif (time_eslap >= 12 and time_eslap <= 20 ):
            cyber_attack = 5
        else:
            cyber_attack = 0
        
        """ Case 3 : random attack  """

        # if time_eslap < 4:
        #     cyber_attack = 0
        # elif 4 <= time_eslap < 16:
        #     if (random.randint(1, 10) > 5):
        #         cyber_attack = -5 + (5 + 5) * random.random()
        #     else:
        #         cyber_attack = 0
        # else:
        #     cyber_attack = 0  # You can uncomment the next line if you want fc to be 3


        """Get Data from sensors """

        _current_speed_ego = get_speed(self._vehicle)/3.6
        
        # get velocity from sensors 
        _current_speed_pre = get_speed(self._vehicle_pre)/3.6

        self._velocity_buffer.append(_current_speed_ego)
        
        
        """ No filter """
        _current_acceleration_ego = get_accel_x(self._vehicle)
        _current_acceleration_pre = get_accel_x(self._vehicle_pre) 

        _current_acceleration_pre = _current_acceleration_pre + cyber_attack
        self._acceleration_ego_buffer.append(_current_acceleration_ego)
        self._acceleration_pre_buffer.append(_current_acceleration_pre)

        _current_acceleration_pre_raw = 0


        """ Apply filter """
        # _current_acceleration_ego_raw = get_accel_x(self._vehicle)
        # _current_acceleration_pre_raw = get_accel_x(self._vehicle_pre) 

        # self._acceleration_ego_buffer.append(_current_acceleration_ego_raw)
        # self._acceleration_pre_buffer.append(_current_acceleration_pre_raw)


        # if (len(self._acceleration_ego_buffer) > 2):
        #     filtered_acceleration_ego = signal.lfilter(self.b, self.a, [self._acceleration_ego_buffer[0],self._acceleration_ego_buffer[1],self._acceleration_ego_buffer[2]])[-1]
        #     _current_acceleration_ego = filtered_acceleration_ego  
        # else :
        #     _current_acceleration_ego = _current_acceleration_ego_raw 

        # if (len(self._acceleration_pre_buffer) > 2):
        #     filtered_acceleration_pre = signal.lfilter(self.b, self.a, [self._acceleration_pre_buffer[0],self._acceleration_pre_buffer[1],self._acceleration_pre_buffer[2]])[-1]
        #     _current_acceleration_pre = filtered_acceleration_pre + cyber_attack  
        # else :
        #     _current_acceleration_pre = _current_acceleration_pre_raw + cyber_attack 



        self._acceleration_buffer.append(_current_acceleration_pre)


        _distance_to_car = compute_distance(self._vehicle.get_location() ,self._vehicle_pre.get_location() )


        velocity_error =   _current_speed_ego - _current_speed_pre
        spacing_error_dot = velocity_error 

        # % % SAACC Model : Error dynamics
        # self.psi = [_distance_to_car - self._l, x_p0(2)-x_f0(2), 0];
        
        # self.psi[0][0] = _distance_to_car - self._l
        # self.psi[1][0] = - velocity_error

        # psi_new = np.matmul(self.Ad,self.psi) + self.Bd*_current_speed_ego + self.Fd*_current_acceleration_pre - self.Wd*cyber_attack + self.deltae
        # self.y = np.matmul(self.Cd,self.psi)
        # self.psi = psi_new

        # using location transform

        self.y[0][0] = _distance_to_car - self._l
        # using radar sensors
        # self.y[0][0] = self._world_sensor.radar_sensor.distance_dected_obj
        self.y[1][0] = - velocity_error

        
        spacing_error = -_distance_to_car + self._L_des

        # Estimate state  
        self.unknow_input_Observer(_current_speed_ego,_current_acceleration_pre)

        cyberest = np.copy(self.ksi_hat[3])
        self.cyber_atk_buffer.append(cyberest)
        """ Apply EMA smoothing for f_c estimate """
        # cyberest =   self.alpha * cyberest + (1- self.alpha)* self.cyber_atk_buffer[0]
        """ Apply filter for f_c estimate """

        if (len(self.cyber_atk_buffer) > 1):
            cyberest = signal.lfilter(self.b_2, self.a_2, [self.cyber_atk_buffer[0],self.cyber_atk_buffer[1]])[-1]  
        else :
            cyberest =   self.alpha * cyberest + (1- self.alpha)* self.cyber_atk_buffer[0]
 

        # Extract the value estimate to easy readable 
        x_hat_0 = self.ksi_hat[0]
        x_hat_1 = self.ksi_hat[1]
        x_hat_2 = self.ksi_hat[2]

        
        if (time_eslap > 5 and abs(cyber_attack) > 2):
            u_syn = (self._h * self._K_1 * self._K_2 * _current_acceleration_pre) - self._K_2*_current_speed_ego - (self._K_1+self._h * self._K_1*self._K_2)*(x_hat_2)+\
                        self._K_3/self._h*(x_hat_1) + self._K_2/self._h*(x_hat_0) - self._K_2/self._h*(self._L_des - self._l) - (self._K_1+self._h * self._K_1*self._K_2)*cyberest    
        else:
            u_syn = (-self._K_1 * _current_acceleration_pre) + (self._K_1 + self._h*self._K_1*self._K_2)*_current_acceleration_ego - \
                            (1/self._h)*(1 - self._h*self._K_1*self._K_2)*spacing_error_dot - (self._K_2/self._h)*spacing_error - self._K_2*_current_speed_ego



        if debug:
            
            # print(f'time  : {self.time_now - time.time()}')
            # self.time_now = time.time()
            # 
            # print("l in carla" + str(self._vehicle.bounding_box.extent.x + self._vehicle_pre.bounding_box.extent.x))

            # print(f'distance : {self._world_sensor.radar_sensor.distance_dected_obj} , {self.y[0][0]}' )
            # print(f'velocity : {(-self._world_sensor.radar_sensor.velocity_dected_obj)*_current_speed_ego} , {_current_speed_pre} ')



            if (time_eslap < 20):
                self.log.write_data(f"t:{time_eslap} fc:{cyberest} , {cyber_attack} psi1:{x_hat_0} , {self.y[0][0]} psi2:{x_hat_1} , {self.y[1][0]} psi3:{x_hat_2} , {_current_acceleration_pre - cyber_attack - _current_acceleration_ego}" )
                self.log.write_data_control(f"control : {u_syn},a_filt_atk : {_current_acceleration_pre},a_filt : {_current_acceleration_pre - cyber_attack},a_raw : {_current_acceleration_pre_raw}")
            else:
                self.log.__del__()

        self.frame += 1  

        u_syn = u_syn/10

        # if (time_eslap<8):
        # u_syn = 0

        return np.clip(u_syn, -1.0, 1.0)
    

    def unknow_input_Observer(self,current_speed_ego ,current_acceleration_pre):
        # %     Unknown input linear descriptor observer
        if self.frame <= 1:
            # intial condition , the vehicle is start at initial condition 
            w_new = np.matmul(self.N,self.w) + np.matmul(self.L,self.y) + (self.M*0) + (self.G*0) + np.matmul(self.Pz,self.deltae)
            _ksi_hat = self.w + np.matmul(self.Qz,self.y)

        else:
            w_new = np.matmul(self.N,self.w) + np.matmul(self.L,self.y) + (self.M*self._velocity_buffer[0]) + (self.G*self._acceleration_buffer[0]) + np.matmul(self.Pz,self.deltae)
            _ksi_hat = self.w + np.matmul(self.Qz,self.y)

        self.ksi_hat[0] = min(max(_ksi_hat[0][0], -30.00), 30.00)
        self.ksi_hat[1] = min(max(_ksi_hat[1][0], -20.00), 20.00)
        self.ksi_hat[2] = min(max(_ksi_hat[2][0], -20.00), 20.00)
        self.ksi_hat[3] = min(max(_ksi_hat[3][0], -50.00), 30.00)

        self.w = w_new


    def LMI(self):
        # Define the variables

        # Calculate g_invEC
        EC = np.vstack((self.E, self.Ce))
        g_invEC = np.linalg.inv(EC.T @ EC) @ EC.T

        # Extract Pz and Qz from g_invEC
        self.Pz = g_invEC[:4, :3]
        self.Qz = g_invEC[:, 3:5]

        # Calculate M and G
        self.M = self.Pz @ self.Be
        self.G = self.Pz @ self.Fe

        # print(f"Pz : {self.Pz}")
        # print(f"Qz : {self.Qz}")
        # print(f"M : {self.M}")
        # print(f"G : {self.G}")

        # Observer gain computation
        p = 2  # Number of outputs
        Az = self.Pz @ self.Ae
        nx = Az.shape[0]  # Length of Az

        print(f"nx : {nx}")


        nx = 4 # Define nx, the size of your state vector
        p = 2  # Define p, the size of your control input vector

        P = cp.Variable((nx, nx), symmetric=True)
        X = cp.Variable((nx, p))
        gamma = cp.Variable(1)
        alpha = 0.5
        # Az = Pz*Ae  # Define your Az matrix
        # Ce =  # Define your Ce matrix
        # Qz =  # Define your Qz matrix

        # Define the LMI constraints

        LMI1 = cp.bmat([
            [-gamma*P , Az.T @ P - self.Ce.T @ X.T],
            [P@Az  - X @ self.Ce, -P]
        ])
        # LMI1 = cp.bmat([
        #     [Az.T @ P @ Az - Az.T @ X @ Ce - Ce.T @ X.T @ Az - P, X @ Ce],
        #     [Ce.T @ X.T, -P]
        # ])

        # LMI2 = cp.bmat([
        #     [-alpha * np.eye(p), X],
        #     [X, -P]
        # ])

        # Define the problem as a semidefinite program
        constraints = [P >> 0, LMI1 << 0]

        # Solve the problem
        problem = cp.Problem(cp.Minimize(gamma), constraints)
        problem.solve()

        # Extract the solution
        Ps = P.value
        Xs = X.value

        print(f"Ps : {Ps}")
        print(f"Xs : {Xs}")

        K = np.linalg.inv(Ps) @ Xs
        self.N = Az - K @ self.Ce
        # poles = np.linalg.eigvals(N)
        self.L = K + self.N@ self.Qz

    # def pi_observer(self,x):
    #     # % Observer
    #     xr = [x(1) x(2) x(3) x(4) x(5)]';
    #     nx = 4;

    #     z = xr(1:nx,1);
    #     f_hat = xr(5,1);
    #     xhat = (z+Q*yn_L);

    #     Ce = Cz(:,1:nx);
    #     N = Az(1:nx,1:nx)-Li(1:4,:)*Ce;
    #     L = Li(1:4,:) + N*Q;
    #     M = Pz*B;
    #     T = -Pz*F;
    #     G = Pz*F;
    #     del = [0 0 k5*(L_des-li_1)/(tau*h)]';

    #     dx1 = N*z + L*yn_L + M*vi + G*mu + T*f_hat + Pz*del;
    #     dx2 = Li[5,:]*(yn_L - Ce*xhat);

    #     self.dx[0]  = dx1
    #     self.dx[1] = dx2
    #     pass

    # def model_SAACC_PIO(self):
    #     % Error dynamics system
    #     A =np.matrix( [0           1                     0;
    #         0           0                     1;
    #         -k5/(tau*h) -(1-k1*k5*h)/(tau*h) (-1+k1+k1*k5*h)/(tau)];
    #     B = [0;
    #         0;
    #         k5/tau];
    #     F = [0;
    #         0;
    #         (1-h*k1*k5)/tau];
    #     C = [1 0 0;
    #         0 1 0];

    #     % Descriptor system
    #     nx = length(A);
    #     na = length(F(1,:));
    #     nw = length(D(1,:));
    #     np = length(C(:,1));
    #     E = [eye(nx) zeros(nx,nw)];
    #     Ae = [A zeros(nx,nw)];
    #     Be = B;
    #     Ce =[C D];

    #     % Observer system
    #     size_E = size(E);
    #     EC = [E; Ce];
    #     PQ = inv(EC'*EC)*EC';
    #     size_PQ = max(size(PQ));
    #     Pz = PQ(:,1:size_E(1,1));
    #     Q = PQ(:,size_E(1,1)+1:size_PQ);

    #     Az = [Pz*Ae -Pz*F;
    #         zeros(nw,nx+nw) zeros(nw,na)];
    #     Cz = [Ce zeros(np,nw)];

        

class PIDLateralController():
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, offset=0, K_P=1.0, K_I=0.0, K_D=0.0, dt=0.03):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param offset: distance to the center line. If might cause issues if the value
                is large enough to make the vehicle invade other lanes.
            :param K_P: Proportional term
            :param K_D: Differential term
            :param K_I: Integral term
            :param dt: time differential in seconds
        """
        self._vehicle = vehicle
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._offset = offset
        self._e_buffer = deque(maxlen=10)

    def run_step(self, waypoint):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoin.

            :param waypoint: target waypoint
            :return: steering control in the range [-1, 1] where:
            -1 maximum steering to left
            +1 maximum steering to right
        """
        return self._pid_control(waypoint, self._vehicle.get_transform())

    def _pid_control(self, waypoint, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

            :param waypoint: target waypoint
            :param vehicle_transform: current transform of the vehicle
            :return: steering control in the range [-1, 1]
        """
        # Get the ego's location and forward vector
        ego_loc = vehicle_transform.location
        v_vec = vehicle_transform.get_forward_vector()
        v_vec = np.array([v_vec.x, v_vec.y, 0.0])

        # Get the vector vehicle-target_wp
        if self._offset != 0:
            # Displace the wp to the side
            w_tran = waypoint.transform
            r_vec = w_tran.get_right_vector()
            w_loc = w_tran.location + carla.Location(x=self._offset*r_vec.x,
                                                         y=self._offset*r_vec.y)
        else:
            w_loc = waypoint.transform.location

        w_vec = np.array([w_loc.x - ego_loc.x,
                          w_loc.y - ego_loc.y,
                          0.0])

        wv_linalg = np.linalg.norm(w_vec) * np.linalg.norm(v_vec)
        if wv_linalg == 0:
            _dot = 1
        else:
            _dot = math.acos(np.clip(np.dot(w_vec, v_vec) / (wv_linalg), -1.0, 1.0))
        _cross = np.cross(v_vec, w_vec)
        if _cross[2] < 0:
            _dot *= -1.0

        self._e_buffer.append(_dot)
        if len(self._e_buffer) >= 2:
            _de = (self._e_buffer[-1] - self._e_buffer[-2]) / self._dt
            _ie = sum(self._e_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip((self._k_p * _dot) + (self._k_d * _de) + (self._k_i * _ie), -1.0, 1.0)

    def change_parameters(self, K_P, K_I, K_D, dt):
        """Changes the PID parameters"""
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt