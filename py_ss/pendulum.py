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
        g = N('g')
        l = N('l')

        with self:
            blocks.Integrator('dphi', iport='d2phi', oport=dphi)
            blocks.Integrator('phi', iport=dphi, oport=phi, x0=np.pi/4)
            blocks.Function('sin(phi)', act_func=lambda t, x: np.sin(x),
                            iport=phi)
            blocks.MulDiv('-g/l', operations='**/',
                          iports=['-', g, l], oport='d2phi', initial=-1)

def main():
    parameters = {
        'l': 0.1,
        'g': 9.81,
        }

    model = SSModel()
    history = helper.run(model, T=np.arange(0.0, 5.0, 0.01),
                         parameters=parameters)
    
    plt.plot(history['t'], history['phi'])
    plt.show()
    
if __name__ == '__main__':
    main()
