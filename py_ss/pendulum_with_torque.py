#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks, helper
from blocks import N as N

class SSModel(blocks.Submodel):
    def __init__(self):
        super().__init__('pendulum')

        phi = N('phi')
        dphi = N('dphi')
        tau = N('tau')
        m = N('m')
        g = N('g')
        l = N('l')

        with self:
            blocks.Integrator('dphi', iport='d2phi', oport=dphi, x0=np.pi/4)
            blocks.Integrator('phi', iport=dphi, oport=phi)
            blocks.Function('sin(phi)', act_func=lambda t, x: np.sin(x),
                            iport=phi)
            blocks.MulDiv('g/l', operations='**/', iports=['-', g, l],
                          oport=-1)
            blocks.MulDiv('', '*///', iports=[tau, m, l, l], oport=-2)
            blocks.AddSub('', '+-', [-2, -1], 'd2phi')

def main():
    parameters = {
        'm': 0.2,
        'l': 0.1,
        'g': 9.81,
        'tau': 0.13,
        }

    model = SSModel()
    history = helper.run(model, T=np.arange(0.0, 5.0, 0.01),
                         parameters=parameters)

    plt.figure()    
    plt.subplot(2, 1, 1); plt.plot(history['t'], history['phi'])
    plt.ylabel('phi')
    plt.subplot(2, 1, 2); plt.plot(history['t'], history['dphi'])
    plt.xlabel('t'); plt.ylabel('dphi')
    plt.show()

if __name__ == '__main__':
    main()
