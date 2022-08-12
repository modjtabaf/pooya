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

        self._dphi    = blocks.Integrator(np.pi/4, 'dphi', 'd2phi', 'dphi')
        self._phi     = blocks.Integrator(0.0, 'phi', 'dphi', 'phi')
        self._sin_phi = blocks.Sin('sin(phi)', 'phi', 'sin_phi')
        self._gain1    = blocks.MulDiv('g/l', '**/', ['sin_phi', 'g', 'l'],
                                      'rhs')
        self._gain2    = blocks.MulDiv('', '*///', ['tau', 'm', 'l', 'l'],
                                      'tau_scaled')
        self._add     = blocks.AddSub('', '+-', ['tau_scaled', 'rhs'], 'd2phi')

        self.add_component(self._dphi)
        self.add_component(self._phi)
        self.add_component(self._sin_phi)
        self.add_component(self._gain1)
        self.add_component(self._gain2)
        self.add_component(self._add)

if __name__ == '__main__':
    model = SSModel()
    states = []
    model.get_states(states)

    parameters = {
        'm': 0.2,
        'l': 0.1,
        'g': 9.81,
        'tau': 0.13,
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
    
    History = {state._state: [state._value] for state in states}
    History['t'] = [t]

    k = 0
    while t < 5.0:
        x = solver.rk4(solver_callback, t0=t, x0=x, h=h)
        k += 1
        t = k*h
        print(k, t)

        model.update_state(t, {k: v for k, v in zip(state_names, x)})

        History['t'].append(t)
        for name, value in zip(state_names, x):
            History[name].append(value)

    plt.figure()    
    plt.subplot(2, 1, 1); plt.plot(History['t'], History['phi'])
    plt.ylabel('phi')
    plt.subplot(2, 1, 2); plt.plot(History['t'], History['dphi'])
    plt.xlabel('t'); plt.ylabel('dphi')
    plt.show()
