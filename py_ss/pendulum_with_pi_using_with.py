#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

import blocks
from blocks import Node as N
import solver

class Pendulum(blocks.Submodel):
    def __init__(self, inport, outport):
        blocks.Submodel.__init__(self, 'pendulum', [inport], [outport])

        sig_tau  = 'tau'
        sig_phi  = 'phi'
        sig_dphi = 'dphi'
        m = N('m')
        l = N('l')
        g = N('g')

        with self:
            blocks.Signal('in', N(inport), sig_tau)
            blocks.MulDiv('', '*///', [sig_tau, m, l, l], 1)
            blocks.AddSub('', '+-', [1, 5], 2)
            blocks.Integrator(np.pi/4, 'dphi', 2, sig_dphi)
            blocks.Integrator(0.0, 'phi', sig_dphi, sig_phi)
            blocks.Function('sin(phi)', sig_phi, 4, lambda t, x: np.sin(x))
            blocks.MulDiv('g/l', '**/', [4, g, l], 5)
            blocks.Signal('out', sig_phi, N(outport))

class PI(blocks.Submodel):
    def __init__(self, Kp, Ki, x0, inport, outport):
        blocks.Submodel.__init__(self, 'PI', [inport], [outport])

        sig_ix   = 'ix'
        sig_xKp  = 'xKp'
        sig_ixKi = 'ixKi'

        with self:
            blocks.Gain(Kp, N('Kp'), N(inport), sig_xKp)
            blocks.Integrator(x0, 'ix', N(inport), sig_ix)
            blocks.Gain(Ki, N('Ki'), sig_ix, sig_ixKi)
            blocks.AddSub('', '++', [sig_xKp, sig_ixKi], N(outport))

class SSModel(blocks.MainModel):
    def __init__(self):
        blocks.MainModel.__init__(self, '')

        sig_phi = N('phi')
        sig_tau = N('tau')
        sig_err = N('err')

        with self:
            blocks.AddSub('', '+-', [N('des_phi'), sig_phi], sig_err)
            PI(40.0, 20.0, 0.0, sig_err, sig_tau)
            Pendulum(sig_tau, sig_phi)

def main():
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

    print(history.keys())
    plt.figure()
    plt.subplot(3, 1, 1); plt.plot(history['t'], history['phi'])
    plt.ylabel('phi')
    plt.subplot(3, 1, 2); plt.plot(history['t'], history['pendulum.dphi'])
    plt.ylabel('dphi')
    plt.subplot(3, 1, 3); plt.plot(history['t'], history['tau'])
    plt.xlabel('t'); plt.ylabel('tau')
    plt.show()

if __name__ == '__main__':
    main()
