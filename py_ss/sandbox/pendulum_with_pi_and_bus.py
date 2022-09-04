#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt

import blocks
from blocks import N as N

class Model(blocks.MainModel):
    def __init__(self):
        super().__init__('Main')

        # nodes
        x = N('x')
        y = 'y'

        # blocks
        with self:
            blocks.Gain('', 2, iport=x, oport=y)
            blocks.Bus('', iports=[x, y])
            blocks.BusSelector('', signals=x, oports=N('x2'))

def main():
    history = Model().run(
        inputs_cb=lambda t, x: {'x': t},
        t_end=1.0, h= 0.2)

    print(history.keys())
    T = history['t']
    plt.figure()
    plt.subplot(2, 1, 1); plt.plot(T, history['x'])
    plt.ylabel('x')
    plt.subplot(2, 1, 2); plt.plot(T, history['x2'])
    plt.xlabel('t'); plt.ylabel('x2')
    plt.show()

if __name__ == '__main__':
    main()
