#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks, helper
from blocks import N as N

def main():
    blk = blocks.Integrator('', iport=N('xd'), oport=N('x'), x0=1.0)
    history = helper.run(blk, T=np.arange(0, 10, 0.1),
                         inputs_cb=lambda t, x: {'xd': 1 if (t < 3 or t > 7) else 0})#np.sin(np.pi*t/5)})

    T = history['t']
    plt.figure()
    plt.plot(T, history['x'], 'b-')
    plt.plot(T, history['xd'], 'r-')
    plt.ylabel('x')
    plt.ylabel('xd')
    plt.grid()
    plt.show()

if __name__ == '__main__':
    main()
