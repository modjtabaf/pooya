#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

import blocks
from blocks import N as N

class PT(blocks.Submodel):
    def __init__(self, iports, oport):
        super().__init__('PT', iports, [oport])

        # nodes
        y_in, tau, y0 = N(self._iports)
        y_out, = N(self._oports)

        # blocks
        with self:
            blocks.AddSub('',
                          operations='+-',
                          iports=[y_in, y_out])
            blocks.MulDiv('',
                          operations='*/',
                          iports=['-', tau])
            blocks.Integrator('', oport=-1)
            blocks.InitialValue('', iport=y0)
            blocks.AddSub('',
                          operations='++',
                          iports=[-1, '-'],
                          oport=y_out)

class ComputeFrontWheelAngleRightLeftPinpoint(blocks.Submodel):
    def __init__(self, iport, oports):
        super().__init__('ComputeFrontWheelAngleRightLeftPinpoint', [iport], oports)

        # nodes
        front_wheel_angle, = N(self._iports)
        front_wheel_angle_right, front_wheel_angle_left = N(self._oports)
        tractor_wheelbase = N('tractor_wheelbase')
        tractor_Width = N('tractor_Width')

        # blocks
        with self:
            blocks.MulDiv('',
                          operations='*/',
                          iports=[tractor_wheelbase, front_wheel_angle],
                          oport=-1)
            blocks.AddSub('',
                          operations='++',
                          iports=[-1, tractor_Width])
            blocks.MulDiv('',
                          operations='*/',
                          iports=[tractor_wheelbase, '-'],
                          oport=front_wheel_angle_right)
            blocks.Gain('', k=0.5, iport=tractor_Width)
            blocks.AddSub('',
                          operations='+-',
                          iports=[-1, '-'])
            blocks.MulDiv('',
                          operations='*/',
                          iports=[tractor_wheelbase, '-'],
                          oport=front_wheel_angle_left)

class SteeringSystem(blocks.Submodel):
    def __init__(self, iport, oport):
        super().__init__('Steering_System', [iport], [oport])

        # nodes
        ad_DsrdFtWhlAngl_Rq_VD     = N(self._iports[0])
        steering_info              = N(self._oports[0])
        front_wheel_angle          = 'front_wheel_angle'
        front_wheel_angle_rate     = 'front_wheel_angle_rate'
        front_wheel_angle_neg      = 'front_wheel_angle_neg'
        front_wheel_angle_rate_neg = 'front_wheel_angle_rate_neg'
        AxFr_front_right           = 'AxFr_front_right'
        AxFr_front_left            = 'AxFr_front_left'

        # blocks
        with self:
            blocks.MulDiv('MulDiv',
                          operations='**',
                          iports=[ad_DsrdFtWhlAngl_Rq_VD, N('front_wheel_ang_gain')])
            blocks.Delay('Delay',
                         iports=['-', N('front_wheel_ang_delay'), N('front_wheel_ang_init_value')],
                         oport=-2)
            blocks.Function('Clamp',
                            act_func=lambda t, x: max(0.001, min(10, x)),
                            iport=N('front_wheel_ang_t_const'))
            PT(iports=[-2, '-', -2], oport=front_wheel_angle)
            blocks.Derivative('Derivative',
                              iport=front_wheel_angle,
                              oport=front_wheel_angle_rate)
            blocks.Gain('Gain1', k=-1,
                        iport=front_wheel_angle,
                        oport=front_wheel_angle_neg)
            blocks.Gain('Gain2', k=-1,
                        iport=front_wheel_angle_rate,
                        oport=front_wheel_angle_rate_neg)
            ComputeFrontWheelAngleRightLeftPinpoint(
                iport=front_wheel_angle,
                oports=[AxFr_front_right, AxFr_front_left])
            blocks.Bus('Bus',
                       iports=[
                           front_wheel_angle,
                            front_wheel_angle_rate,
                            front_wheel_angle_neg,
                            front_wheel_angle_rate_neg,
                            AxFr_front_right,
                            AxFr_front_left],
                       oport=steering_info)

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
    front_wheel_angle_Rq_data = dat['front_wheel_angle_Rq_data'].ravel()[:len(front_wheel_angle_Rq_t)]

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

    steering_system = SteeringSystem(
        iport='front_wheel_angle_Rq',
        oport='steering_info')
    history = blocks.run_helper(
        steering_system, parameters=parameters, inputs_cb=inputs_cb,
        t0=front_wheel_angle_Rq_t[0], t_end=front_wheel_angle_Rq_t[-1])

    print(history.keys())
    plt.figure()
    plt.plot(history['t'], history['Steering_System.front_wheel_angle'], 'b-')
    plt.plot(mat_time, mean_front_wheel_steering_angle, 'r:')
    plt.xlabel('t'); plt.ylabel('front wheel angle')
    plt.show()
    
if __name__ == '__main__':
    main()
