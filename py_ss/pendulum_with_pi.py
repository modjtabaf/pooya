#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks
from blocks import N as N

class Pendulum(blocks.Submodel):
    def __init__(self, iport, oport):
        super().__init__('pendulum', [iport], [oport])

        # nodes
        tau  = N(iport)
        dphi = N('dphi')
        phi  = N(oport)
        m = N('m')
        l = N('l')
        g = N('g')

        # blocks
        with self:
            blocks.MulDiv('tau/ml2', operations='*///', iports=[tau, m, l, l])
            blocks.AddSub('err', operations='+-', iports=['-', -1])
            blocks.Integrator('dphi', oport=dphi)
            blocks.Integrator('phi', iport=dphi, oport=phi)
            blocks.Function('sin(phi)', act_func=lambda t, x: np.sin(x), iport=phi)
            blocks.MulDiv('g/l', operations='**/', iports=['-', g, l], oport=-1)

class PI(blocks.Submodel):
    def __init__(self, Kp, Ki, iport, oport, x0=0.0):
        super().__init__('PI', [iport], [oport])

        # nodes
        x = N(iport)

        # blocks
        with self:
            blocks.Gain('Kp', k=Kp, iport=x, oport=-1)
            blocks.Integrator('ix', iport=x, x0=x0)
            blocks.Gain('Ki', k=Ki)
            blocks.AddSub('', operations='++', iports=[-1, '-'], oport=N(oport))

class SSModel(blocks.MainModel):
    def __init__(self):
        super().__init__('')

        # nodes
        phi = N('phi')
        tau = N('tau')
        err = N('err')

        # blocks
        with self:
            blocks.AddSub('',
                          operations='+-',
                          iports=[N('des_phi'), phi],
                          oport=err)
            PI(Kp=40.0, Ki=20.0, iport=err, oport=tau)
            Pendulum(iport=tau, oport=phi)

def main():
    parameters = {
        'm': 0.2,
        'l': 0.1,
        'g': 9.81,
        'des_phi': np.pi/4,
        }

    history = SSModel().run(parameters=parameters, t_end=5.0)

    print(history.keys())
    T = history['t']
    plt.figure()
    plt.subplot(3, 1, 1); plt.plot(T, history['phi'])
    plt.ylabel('phi')
    plt.subplot(3, 1, 2); plt.plot(T, history['dphi'])
    plt.ylabel('dphi')
    plt.subplot(3, 1, 3); plt.plot(T, history['tau'])
    plt.xlabel('t'); plt.ylabel('tau')
    plt.show()

if __name__ == '__main__':
    main()
