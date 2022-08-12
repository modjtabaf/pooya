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

class SSModel(blocks.Submodel):
    def __init__(self):
        blocks.Submodel.__init__(self, 'mass_spring')

        self.xd_ = blocks.Integrator(0.1, 'xd', 'xdd', 'xd')
        self.x_  = blocks.Integrator(0.0, 'x', 'xd', 'x')
        self.gain_ = blocks.Gain(-1.0/1.0, '-k/m', 'x', 'xdd')

        self.add_component(self.xd_)
        self.add_component(self.x_)
        self.add_component(self.gain_)

    def process(self, t, x):
        n_processed = self._process(t, x, True)
        while True:
            n = self._process(t, x, False)
            if n == 0:
                break
            n_processed += n

        return n_processed

if __name__ == '__main__':
    model = SSModel()
    states = []
    model.get_states(states)
    
    state_names = [state._state for state in states]

    def solver_callback(t, x):
        x = {k: v for k, v in zip(state_names, x)}
        model.process(t, x)
        return np.array([x[state._deriv] for state in states])

    h = 0.01    
    x = np.array([state._value for state in states])
    t = 0.0
    X = [x]
    T = [t]
    k = 0
    while t < 50.0:
        x = solver.rk4(solver_callback, t0=t, x0=x, h=h)
        k += 1
        t = k*h
        print(k, t)

        model.update_states(t, {k: v for k, v in zip(state_names, x)})

        X.append(x)
        T.append(t)

    plt.plot(T, [x[1] for x in X])
    plt.show()
