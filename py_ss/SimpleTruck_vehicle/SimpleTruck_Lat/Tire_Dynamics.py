#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

import blocks
from blocks import N as N

class AvoidZeroVelocity(blocks.Base):
    def __init__(self, iports, oport):
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
        v_tire_lon = N(self._iports[0])
        Fz         = N(self._iports[1])
        R_dyn      = N(self._iports[2])
        T_roll     = N(self._oports[0])
        F_roll     = N(self._oports[1])

        # blocks
        with self:
            # setting the sign of F_roll with some down time
            blocks.Function('', v_tire_lon, 1, lambda t, x: (np.abs(x) > 1e-2)*np.sign(x))

            blocks.MulDiv('', '***', [Fz, N('tire_roll_resist'), 1], 2)

            # calculation of the time constant which depends on velocity and relaxation length
            blocks.Const('', 0.69, 3)
            AvoidZeroVelocity([v_tire_lon, 3], 4)
            blocks.MulDiv('', '*/', [4, N('tire_relax_length')], 5)

            # low-pass filter
            blocks.AddSub('', '+-', [2, F_roll], 6)
            blocks.MulDiv('', '*/', [6, 5], 7)
            blocks.Integrator('', 7, F_roll)

            blocks.MulDiv('', '**', [R_dyn, F_roll], T_roll)

class TireDynamics(blocks.MainModel):
    def __init__(self):
        super().__init__('Tire_Dynamics', ['axle_torque', 'kinematics'], ['tire_info'])

        # nodes
        ad_DsrdFtWhlAngl_Rq_VD     = N(self._iports[0])
        front_wheel_angle          = N('front_wheel_angle')
        front_wheel_angle_rate     = N('front_wheel_angle_rate')
        front_wheel_angle_neg      = N('front_wheel_angle_neg')
        front_wheel_angle_rate_neg = N('front_wheel_angle_rate_neg')
        AxFr_front_right           = N('AxFr_front_right')
        AxFr_front_left            = N('AxFr_front_left')
        steering_info              = N(self._oports[0])

        # blocks
        with self:
            blocks.MulDiv('MulDiv', '**', [ad_DsrdFtWhlAngl_Rq_VD, N('front_wheel_ang_gain')], 1)
            blocks.Delay('Delay', [1, N('front_wheel_ang_delay'), N('front_wheel_ang_init_value')], 2)
            blocks.Function('Function', N('front_wheel_ang_t_const'), 3, lambda t, x: max(0.001, min(10, x)))
            PT([2, 3, 2], front_wheel_angle)
            blocks.Derivative('Derivative', front_wheel_angle, front_wheel_angle_rate)
            blocks.Gain('Gain1', -1, front_wheel_angle, front_wheel_angle_neg)
            blocks.Gain('Gain2', -1, front_wheel_angle_rate, front_wheel_angle_rate_neg)
            ComputeFrontWheelAngleRightLeftPinpoint(front_wheel_angle, [
                AxFr_front_right, AxFr_front_left])
            blocks.Bus('bus', [
                front_wheel_angle,
                front_wheel_angle_rate,
                front_wheel_angle_neg,
                front_wheel_angle_rate_neg,
                AxFr_front_right,
                AxFr_front_left,
                ], steering_info)
    
def main():
    parameters = {
        'tractor_wheelbase': 5.8325,
        'tractor_Width': 2.5,
        'front_wheel_ang_t_const': 0.1,
        'front_wheel_ang_delay': 0.02,
        'front_wheel_ang_gain': 1.0,
        'front_wheel_ang_init_value': 0.0,
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

    history = Steering_System().run(parameters=parameters, inputs_cb=inputs_cb,
                                    t0=front_wheel_angle_Rq_t[0],
                                    t_end=front_wheel_angle_Rq_t[-1])

    print(history.keys())
    plt.figure()
    plt.plot(history['t'], history['front_wheel_angle'], 'b-')
    plt.plot(mat_time, mean_front_wheel_steering_angle, 'r:')
    plt.xlabel('t'); plt.ylabel('front whee angle')
    plt.show()
    
if __name__ == '__main__':
    main()
