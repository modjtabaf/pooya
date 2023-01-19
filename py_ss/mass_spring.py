#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks, helper
from blocks import N as N

class SSModel(blocks.Submodel):
    def __init__(self):
        super().__init__('')

        x = N('x')
        xd = N('xd')

        with self:
            blocks.Integrator('xd', iport='-xdd', oport=xd, x0=0.1)
            blocks.Integrator('x', iport=xd, oport=x)
            blocks.Gain('-k/m', k=-1.0/1.0, iport=x, oport='-xdd')

def main():
    model = SSModel()
    history = helper.run(model, T=np.arange(0, 5, 0.01))

    T = history['t']
    plt.figure()
    plt.subplot(2, 1, 1); plt.plot(T, history['x'])
    plt.ylabel('x')
    plt.subplot(2, 1, 2); plt.plot(T, history['xd'])
    plt.ylabel('xd')
    plt.show()

if __name__ == '__main__':
    main()
