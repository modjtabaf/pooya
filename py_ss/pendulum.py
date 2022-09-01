#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Wed Aug 10 20:52:40 2022

@author: fathi
"""

import matplotlib.pyplot as plt
import numpy as np

import blocks
import solver

class SSModel(blocks.MainModel):
    def __init__(self):
        blocks.MainModel.__init__(self, 'pendulum')

        self._dphi    = blocks.Integrator('dphi', 'd2phi', 'dphi')
        self._phi     = blocks.Integrator('phi', 'dphi', 'phi', x0=np.pi/4)
        self._sin_phi = blocks.Function('sin(phi)', 'phi', 'sin_phi', lambda t, x: np.sin(x))
        self._gain    = blocks.MulDiv('-g/l', '**/', ['sin_phi', 'g', 'l'],
                                      'd2phi', initial=-1)

        self.add_component(self._dphi)
        self.add_component(self._phi)
        self.add_component(self._sin_phi)
        self.add_component(self._gain)

if __name__ == '__main__':
    model = SSModel()
    states = []
    model.get_states(states)

    parameters = {
        'm': 0.2,
        'l': 0.1,
        'g': 9.81,
        }

    state_names = [state._state for state in states]

    def solver_callback(t, x):
        x = {k: v for k, v in zip(state_names, x)}
        for p, v in parameters.items():
            x[p] = v
        model.process(t, x)
        return np.array([x[state._deriv] for state in states])

    h = 0.01    
    x = np.array([state._value for state in states])
    t = 0.0
    X = [x]
    T = [t]
    k = 0
    while t < 5.0:
        x = solver.rk4(solver_callback, t0=t, x0=x, h=h)
        k += 1
        t = k*h
        print(k, t)

        model.step(t, {k: v for k, v in zip(state_names, x)})

        X.append(x)
        T.append(t)

    plt.plot(T, [x[1] for x in X])
    plt.show()
