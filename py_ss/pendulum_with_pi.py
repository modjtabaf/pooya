#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks
from blocks import N as N

class Pendulum(blocks.Submodel):
    def __init__(self, inport, outport):
        super().__init__('pendulum', [inport], [outport])

        # nodes
        tau  = N(inport)
        dphi = N('dphi')
        phi  = N(outport)
        m = N('m')
        l = N('l')
        g = N('g')

        with self:
            # blocks
            blocks.MulDiv('', '*///', [tau, m, l, l], 1)
            blocks.AddSub('', '+-', [1, 5], 2)
            blocks.Integrator(0.0, 'dphi', 2, dphi)
            blocks.Integrator(0.0, 'phi', dphi, phi)
            blocks.Function('sin(phi)', phi, 4, lambda t, x: np.sin(x))
            blocks.MulDiv('g/l', '**/', [4, g, l], 5)

class PI(blocks.Submodel):
    def __init__(self, Kp, Ki, x0, inport, outport):
        super().__init__('PI', [inport], [outport])

        # nodes
        x = N(inport)

        # blocks
        with self:
            blocks.Gain(Kp, N('Kp'), x, 1)
            blocks.Integrator(x0, 'ix', x, 2)
            blocks.Gain(Ki, N('Ki'), 2, 3)
            blocks.AddSub('', '++', [1, 3], N(outport))

class SSModel(blocks.MainModel):
    def __init__(self):
        super().__init__('')

        # nodes
        phi = N('phi')
        tau = N('tau')
        err = N('err')

        # blocks
        with self:
            blocks.AddSub('', '+-', [N('des_phi'), phi], err)
            PI(40.0, 20.0, 0.0, err, tau)
            Pendulum(tau, phi)

def main():
    parameters = {
        'm': 0.2,
        'l': 0.1,
        'g': 9.81,
        'des_phi': np.pi/4,
        }

    history = SSModel().run(parameters=parameters, t_end=5.0)

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
