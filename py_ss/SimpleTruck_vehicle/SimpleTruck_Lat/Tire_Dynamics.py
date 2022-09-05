#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

import blocks
from blocks import N as N

class AvoidZeroVelocity(blocks.Base):
    def __init__(self, iports, oport='-'):
        super().__init__('avoid_zero_velocity', iports, [oport])

    def activation_function(self, t, x):
        v_lon, v_trans = x
        
        v1 = np.abs(v_lon)
        if v1 >= v_trans:
            v_lon_abs_limited = v1
        else:
            v_lon_abs_limited = 2*v_trans/(3 - (v_lon/v_trans)**2)

        return [v_lon_abs_limited]
    
class RollingResistance(blocks.Submodel):
    def __init__(self, iports, oports):
        super().__init__('Rolling_Resistance', iports, oports)

        # nodes
        v_tire_lon, Fz, R_dyn = N(self._iports)
        T_roll, F_roll = N(self._oports)

        # blocks
        with self:
            # setting the sign of F_roll with some down time
            blocks.Function('',
                            act_func=lambda t, x: (np.abs(x) > 1e-2)*np.sign(x),
                            iport=v_tire_lon)

            blocks.MulDiv('',
                          operations='***',
                          iports=[Fz, N('tire_roll_resist'), '-'],
                          oport=-1)

            # calculation of the time constant which depends on velocity and relaxation length
            blocks.Const('', value=0.69)
            AvoidZeroVelocity(iports=[v_tire_lon, '-'])
            blocks.MulDiv('',
                          operations='*/',
                          iports=['-', N('tire_relax_length')],
                          oport=-2)

            # low-pass filter
            blocks.AddSub('',
                          operations='+-',
                          iports=[-1, F_roll])
            blocks.MulDiv('',
                          operations='*/',
                          iports=['-', -2])
            blocks.Integrator('', oport=F_roll)

            blocks.MulDiv('',
                          operations='**',
                          iports=[R_dyn, F_roll],
                          oport=T_roll)

class VerticalTireDynamics(blocks.Base):
    def __init__(self, oports):
        super().__init__('vertical_tire_dynamics', [], oports)

    def activation_function(self, t, x):
        R_eff = self._known_values['tire_static_radius']

        axle_load_empty = self._known_values['axle_load_empty']
        axle_load_full  = self._known_values['axle_load_full']
        payload_factor  = self._known_values['payload_factor']
        Fz = 9.81*(axle_load_empty + payload_factor*(axle_load_full - axle_load_empty))

        return [R_eff, Fz]
    
class TireLonNoSlip(blocks.Submodel):
    def __init__(self, iports, oports):
        super().__init__('tire_lon_no_slip', iports, oports)

        # nodes
        v_tire_lon, axle_torque, Fz, R_eff = N(self._iports)
        Fx, Omega, kappa = N(self._oports)
        Omega_prev = '-Omega_prev'

        # blocks
        with self:
            blocks.Memory('', iport=Omega, oport=Omega_prev)

            def Fx_act_func(t, x):
                axle_torque, R_eff, Omega_prev = x
                # Fx = axle_torque*float((axle_torque > 0) or (Omega_prev > 0))/R_eff
                Fx = axle_torque/R_eff if ((axle_torque > 0) or (Omega_prev > 0)) else 0.0
                return [Fx]

            blocks.MIMOFunction('',
                            act_func=Fx_act_func,
                            iports=[axle_torque, R_eff, Omega_prev],
                            oports=[Fx])

            def act_func2(t, x):
                R_eff, v_tire_lon = x
                Omega = v_tire_lon/R_eff
                kappa = Omega*0.0
                return [Omega, kappa]

            blocks.MIMOFunction('',
                            act_func=act_func2,
                            iports=[R_eff, v_tire_lon],
                            oports=[Omega, kappa])

class TireLatBasicSideSlip(blocks.Submodel):
    def __init__(self, iports, oports):
        super().__init__('tire_lat_basic_side_slip', iports, oports)

        # nodes
        v_tire_lon, v_tire_lat, Fz = N(self._iports)
        Fy, alpha = N(self._oports)
        tire_relax_length = N('tire_relax_length')

        # blocks
        with self:
            blocks.Const('v_trans', value=10.0)

            AvoidZeroVelocity(iports=[v_tire_lon, '-'])

            blocks.MulDiv('',
                          operations='*/*',
                          iports=['-', tire_relax_length, alpha],
                          oport=-1,
                          initial=-1.0)

            blocks.MulDiv('',
                          operations='/*',
                          iports=[tire_relax_length, v_tire_lat],
                          oport=-2)

            blocks.AddSub('',
                          operations='++',
                          iports=[-1, -2])

            blocks.Integrator('', oport=alpha)

            blocks.MulDiv('',
                          operations='***',
                          iports=[N('n_tires'), N('tire_c_lapha'), alpha],
                          oport=Fy,
                          initial=-1.0)

class WheelSpeed(blocks.Submodel):
    def __init__(self, iports, oports):
        super().__init__('wheel_speed', iports, oports)

        # nodes
        x, rz = N(self._iports)
        FL_x, FR_x, RL_x, RR_x = N(self._oports)

        # blocks
        with self:
            def wheel_speed_FL(t, x):
                yaw, vel_x = x
                tractor_Width = self._known_values['tractor_Width']
                WheelSpeed_FL_x = vel_x - 0.5*tractor_Width*yaw
                return [WheelSpeed_FL_x]

            blocks.MIMOFunction('wheel_speed_FL',
                            act_func=wheel_speed_FL,
                            iports=[rz, x],
                            oports=[FL_x])

            def wheel_speed_FR(t, x):
                yaw, vel_x = x
                tractor_Width = self._known_values['tractor_Width']
                WheelSpeed_FR_x = vel_x + 0.5*tractor_Width*yaw
                return [WheelSpeed_FR_x]

            blocks.MIMOFunction('wheel_speed_FR',
                            act_func=wheel_speed_FR,
                            iports=[rz, x],
                            oports=[FR_x])

            def wheel_speed_RL(t, x):
                yaw, vel_x = x
                tractor_Width = self._known_values['tractor_Width']
                WheelSpeed_RL_x = vel_x - 0.5*tractor_Width*yaw
                return [WheelSpeed_RL_x]

            blocks.MIMOFunction('wheel_speed_RL',
                            act_func=wheel_speed_RL,
                            iports=[rz, x],
                            oports=[RL_x])

            def wheel_speed_RR(t, x):
                yaw, vel_x = x
                tractor_Width = self._known_values['tractor_Width']
                WheelSpeed_RR_x = vel_x + 0.5*tractor_Width*yaw
                return [WheelSpeed_RR_x]

            blocks.MIMOFunction('wheel_speed_RR',
                            act_func=wheel_speed_RR,
                            iports=[rz, x],
                            oports=[RR_x])

class TireDynamics(blocks.Submodel):
    def __init__(self, iports, oport):
        super().__init__('Tire_Dynamics', iports, oport)

        # nodes
        axle_torque, kinematics = N(self._iports)
        tire_info, = N(self._oports)
        R_eff = 'R_eff'
        Fz = 'Fz'
        T_roll = 'T_roll'
        F_roll = 'F_roll'
        v_tire_lon = 'v_tire_lon'
        Fx = 'Fx'
        Omega = 'Omega'
        kappa = 'kappa'
        tire_lon = 'tire_lon'
        tire_lat = 'tire_lat'
        Fy = 'Fy'
        alpha = 'alpha'
        front_wss_left_x = 'front_wss_left_x'
        front_wss_right_x = 'front_wss_right_x'
        rear_wss_right_x = 'rear_wss_right_x'
        rear_wss_left_x = 'rear_wss_left_x'

        # blocks
        with self:
            VerticalTireDynamics(oports=[R_eff, Fz])
            
            blocks.BusSelector('', iport=kinematics, oports=v_tire_lon)
            RollingResistance(iports=[v_tire_lon, Fz, R_eff], oports=[T_roll, F_roll])

            blocks.AddSub('', operations='+-', iports=[axle_torque, T_roll])
            TireLonNoSlip(
                iports=[v_tire_lon, '-', Fz, R_eff],
                oports=[Fx, Omega, kappa])

            blocks.BusSelector('', iport=kinematics, oports=[tire_lon, tire_lat])
            TireLatBasicSideSlip(iports=[tire_lon, tire_lat, Fz],
                                 oports=[Fy, alpha])

            blocks.BusSelector('', iport=kinematics, signals=['vehicle', 'trailer'])
            blocks.BusSelector('', iport='vehicle', signal='x', oports='vx')
            blocks.BusSelector('', iport='trailer', signal='rz', oports='rz')
            WheelSpeed(
                iports=['vx', 'rz'],
                oports=[
                    front_wss_left_x,
                    front_wss_right_x,
                    rear_wss_right_x,
                    rear_wss_left_x])

            blocks.Bus('bus', [
                Fx, Fy, Fz, Omega, R_eff, kappa, alpha, T_roll, F_roll,
                front_wss_left_x, front_wss_right_x,
                rear_wss_right_x, rear_wss_left_x],
                tire_info)

def main():
    parameters = {
        'axle_load_empty':    np.array([5060., 6000., 4110.]),
        'axle_load_full':     np.array([5240., 13280., 16280.]),
        'payload_factor':     1.0,
        'tire_static_radius': np.array([0.475, 0.51, 0.475]),
        }

    dat = loadmat('/home/fathi/torc/git/playground/py_ss/data/matlab.mat')
    front_wheel_angle_Rq_t = dat['front_wheel_angle_Rq_t'].ravel()[:5000]
    front_wheel_angle_Rq_data = dat['front_wheel_angle_Rq_data'].ravel()[:5000]

    mat_time = dat['simpletruck_analyzer_results'][0, 0][0]
    mean_front_wheel_steering_angle = dat['simpletruck_analyzer_results'][0, 0][29][0, 0][0]
    # for k in range(1, 81):
    #     print(k, dat['simpletruck_analyzer_results'][0, 0][k][0, 0][2])
    del dat

    def inputs_cb(t, x):
        front_wheel_angle_Rq = np.interp(t, front_wheel_angle_Rq_t,
                                         front_wheel_angle_Rq_data)
        inputs = {
            'front_wheel_angle_Rq': front_wheel_angle_Rq,
            }
        
        return inputs

    tire_dynamics = TireDynamics(
        iports=['axle_torque', 'kinematics'],
        oport='tire_info')
    history = blocks.run_helper(tire_dynamics,
        parameters=parameters, inputs_cb=inputs_cb,
        t0=front_wheel_angle_Rq_t[0],
        t_end=0.011)#front_wheel_angle_Rq_t[-1])

    print(history.keys())
    plt.figure()
    plt.plot(history['t'], history['front_wheel_angle'], 'b-')
    plt.plot(mat_time, mean_front_wheel_steering_angle, 'r:')
    plt.xlabel('t'); plt.ylabel('front whee angle')
    plt.show()
    
if __name__ == '__main__':
    main()
