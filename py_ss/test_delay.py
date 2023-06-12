#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks, helper, solver
from blocks import N as N

class SSModel(blocks.Submodel):
    def __init__(self):
        super().__init__('')

        x = N('x')
        xd = N('xd')

        with self:
            blocks.Const('', 1.0, oport='-time_delay')
            blocks.Const('', 0.0, oport='-initial')
            blocks.Delay('', iports=[x, '-time_delay', '-initial'], oport=xd)

def main():
    model = SSModel()
    history = helper.run(model, T=np.arange(0, 10, 1),
                         inputs_cb=lambda t, x: {'x': np.sin(np.pi*t/5)},
                         stepper=solver.passive)

    T = history['t']
    plt.figure()
    plt.plot(T, history['x'], 'b-')
    plt.plot(T, history['xd'], 'r:')
    plt.ylabel('x')
    plt.ylabel('xd')
    plt.show()

if __name__ == '__main__':
    main()
