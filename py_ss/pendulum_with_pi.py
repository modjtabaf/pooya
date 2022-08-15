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

class Pendulum(blocks.Submodel):
    def __init__(self, inport, outport):
        blocks.Submodel.__init__(self, 'pendulum', [inport], [outport])

        sig_tau        = self.make_signal_name('tau')
        sig_tau_scaled = self.make_signal_name('tau_scaled')
        sig_phi        = self.make_signal_name('phi')
        sig_dphi       = self.make_signal_name('dphi')
        sig_d2phi      = self.make_signal_name('d2phi')
        sig_sin_phi    = self.make_signal_name('sin_phi')

        self._insig   = blocks.Signal('in', inport, sig_tau)
        self._gain1   = blocks.MulDiv('', '*///', [sig_tau, 'm', 'l', 'l'],
                                      sig_tau_scaled)
        self._add     = blocks.AddSub('', '+-', [sig_tau_scaled, 'rhs'], sig_d2phi)
        self._dphi    = blocks.Integrator(np.pi/4, 'dphi', sig_d2phi, sig_dphi)
        self._phi     = blocks.Integrator(0.0, 'phi', sig_dphi, sig_phi)
        self._sin_phi = blocks.Sin('sin(phi)', sig_phi, sig_sin_phi)
        self._gain2   = blocks.MulDiv('g/l', '**/', [sig_sin_phi, 'g', 'l'],
                                      'rhs')
        self._outsig  = blocks.Signal('out', sig_phi, outport)

        self.add_component(self._insig)
        self.add_component(self._gain1)
        self.add_component(self._add)
        self.add_component(self._dphi)
        self.add_component(self._phi)
        self.add_component(self._sin_phi)
        self.add_component(self._gain2)
        self.add_component(self._outsig)

class PI(blocks.Submodel):
    def __init__(self, Kp, Ki, x0, inport, outport):
        blocks.Submodel.__init__(self, 'PI', [inport], [outport])
        self._Kp = Kp
        self._Ki = Ki

        sig_ix   = self.make_signal_name('ix')
        sig_xKp  = self.make_signal_name('xKp')
        sig_ixKi = self.make_signal_name('ixKi')

        self.add_component(blocks.Gain(Kp, 'Kp', inport, sig_xKp))
        self.add_component(blocks.Integrator(x0, 'ix', inport, sig_ix))
        self.add_component(blocks.Gain(Ki, 'Ki', sig_ix, sig_ixKi))
        self.add_component(blocks.AddSub('', '++', [sig_xKp, sig_ixKi], outport))

class SSModel(blocks.MainModel):
    def __init__(self):
        blocks.MainModel.__init__(self, 'pendulum_with_pid')

        sig_phi = 'phi'
        sig_tau = 'tau'
        sig_err = 'err'

        self.add_component(blocks.AddSub('', '+-', ['des_phi', sig_phi], sig_err))
        self.add_component(PI(40.0, 20.0, 0.0, sig_err, sig_tau))
        self.add_component(Pendulum(sig_tau, sig_phi))

if __name__ == '__main__':
    model = SSModel()
    states = []
    model.get_states(states)

    parameters = {
        'm': 0.2,
        'l': 0.1,
        'g': 9.81,
        'des_phi': np.pi/4,
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
    
    history = {}

    def update_history(t, x):
        y = {k: v for k, v in zip(state_names, x)}
        for p, v in parameters.items():
            y[p] = v
        model.process(t, y)
        model.update_state(t, y)

        if not history:
            history['t'] = []
            for k in y.keys():
                if k not in parameters:
                    history[k] = []

        history['t'].append(t)
        for k, v in y.items():
            if k not in parameters:
                history[k].append(v)

    update_history(t, x)

    k = 0
    while t < 5.0:
        x = solver.rk4(solver_callback, t0=t, x0=x, h=h)
        k += 1
        t = k*h
        print(k, t)

        update_history(t, x)

    plt.figure()
    plt.subplot(3, 1, 1); plt.plot(history['t'], history['pendulum.phi'])
    plt.ylabel('phi')
    plt.subplot(3, 1, 2); plt.plot(history['t'], history['pendulum.dphi'])
    plt.ylabel('dphi')
    plt.subplot(3, 1, 3); plt.plot(history['t'], history['tau'])
    plt.xlabel('t'); plt.ylabel('tau')
    plt.show()
