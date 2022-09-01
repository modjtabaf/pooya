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
        super().__init__('')

        with self:
            blocks.Integrator('xd', 'xdd', 'xd', x0=0.1)
            blocks.Integrator('x', 'xd', 'x')
            blocks.Gain('-k/m', -1.0/1.0, 'x', 'xdd')

def main():
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

    history = {}

    def update_history(t, x):
        y = {k: v for k, v in zip(state_names, x)}
        model.process(t, y)
        model.step(t, y)

        if not history:
            history['t'] = []
            for k in y.keys():
                history[k] = []

        history['t'].append(t)
        for k, v in y.items():
            history[k].append(v)

    update_history(t, x)

    k = 0
    while t < 5.0:
        x = solver.rk4(solver_callback, t0=t, x0=x, h=h)
        k += 1
        t = k*h
        print(k, t)

        update_history(t, x)

    T = history['t']
    plt.figure()
    plt.subplot(2, 1, 1); plt.plot(T, history['x'])
    plt.ylabel('x')
    plt.subplot(2, 1, 2); plt.plot(T, history['xd'])
    plt.ylabel('xd')
    plt.show()

if __name__ == '__main__':
    main()
