#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks, helper, solver
from blocks import N as N

def main():
    memory = blocks.Memory('', iport=N('x'), oport=N('xd'))
    history = helper.run(
        memory, T=np.arange(0, 10, 0.1), inputs_cb=lambda t, x: {'x': np.sin(np.pi*t/5)},
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
