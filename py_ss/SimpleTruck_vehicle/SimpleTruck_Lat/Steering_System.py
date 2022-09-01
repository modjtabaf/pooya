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
        y_in  = N(self._iports[0])
        tau   = N(self._iports[1])
        y0    = N(self._iports[2])
        y_out = N(self._oports[0])

        # blocks
        with self:
            blocks.AddSub('AddSub', operations='+-', iports=[y_in, y_out])
            blocks.MulDiv('MulDiv', operations='*/', iports=['-', tau])
            blocks.Integrator('', oport=-1)
            blocks.InitialValue('IV', iport=y0)
            blocks.AddSub('AddSub', operations='++', iports=[-1, '-'], oport=y_out)

class ComputeFrontWheelAngleRightLeftPinpoint(blocks.Submodel):
    def __init__(self, iport, oports):
        super().__init__('ComputeFrontWheelAngleRightLeftPinpoint', [iport], oports)

        # nodes
        front_wheel_angle       = N(iport)
        front_wheel_angle_right = N(oports[0])
        front_wheel_angle_left  = N(oports[1])
        tractor_wheelbase = N('tractor_wheelbase')
        tractor_Width = N('tractor_Width')

        # blocks
        with self:
            blocks.MulDiv('MulDiv1', operations='*/', iports=[tractor_wheelbase, front_wheel_angle], oport=-1)
            blocks.AddSub('AddSub1', operations='++', iports=[-1, tractor_Width])
            blocks.MulDiv('MulDiv2', operations='*/', iports=[tractor_wheelbase, '-'], oport=front_wheel_angle_right)
            blocks.Gain('Gain', k=0.5, iport=tractor_Width)
            blocks.AddSub('AddSub2', operations='+-', iports=[-1, '-'])
            blocks.MulDiv('MulDiv3', operations='*/', iports=[tractor_wheelbase, '-'], oport=front_wheel_angle_left)

class SteeringSystem(blocks.MainModel):
    def __init__(self, iport, oport):
        super().__init__('Steering_System', [iport], [oport])

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
            blocks.MulDiv('MulDiv',
                          operations='**',
                          iports=[ad_DsrdFtWhlAngl_Rq_VD, N('front_wheel_ang_gain')])
            blocks.Delay('Delay',
                         iports=['-', N('front_wheel_ang_delay'), N('front_wheel_ang_init_value')],
                         oport=-2)
            blocks.Function('Function',
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
            blocks.Bus('bus',
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

    history = SteeringSystem(iport='front_wheel_angle_Rq', oport='steering_info').run(
        parameters=parameters, inputs_cb=inputs_cb,
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
