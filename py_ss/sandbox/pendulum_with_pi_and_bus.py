#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt

import blocks
from blocks import N as N
import solver

class Model(blocks.MainModel):
    def __init__(self):
        super().__init__('Main')

        # nodes
        x = N('x')
        z = N('z')

        # blocks
        with self:
            blocks.Gain('', 2, iport=x)
            blocks.Bus('', iports=[x, '-'])
            blocks.BusSelector('', signals=x, oports=z)

def main():
    model = Model()
    history = model.run(
        inputs_cb=lambda t, x: {'x': t},
        t_end=1.0, h= 0.2, stepper=solver.passive)

    print(history.keys())
    T = history['t']
    plt.figure()
    plt.subplot(2, 1, 1); plt.plot(T, history['x'])
    plt.ylabel('x')
    plt.subplot(2, 1, 2); plt.plot(T, history['z'])
    plt.xlabel('t'); plt.ylabel('z')
    plt.show()

if __name__ == '__main__':
    main()
