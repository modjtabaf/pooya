#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Wed Aug 10 20:52:40 2022

@author: fathi
"""

import matplotlib.pyplot as plt
import numpy as np
import bisect
import pyglet
from pyglet import shapes

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

        self.add_component(blocks.Signal('in', inport, sig_tau))
        self.add_component(blocks.MulDiv('', '*///', [sig_tau, 'm', 'l', 'l'],
                                      sig_tau_scaled))
        self.add_component(blocks.AddSub('', '+-', [sig_tau_scaled, 'rhs'], sig_d2phi))
        self.add_component(blocks.Integrator(np.pi/4, 'dphi', sig_d2phi, sig_dphi))
        self.add_component(blocks.Integrator(0.0, 'phi', sig_dphi, sig_phi))
        self.add_component(blocks.Function('sin(phi)', sig_phi, sig_sin_phi,
                                           act_func=(lambda t, x: np.sin(x))))
        self.add_component(blocks.MulDiv('g/l', '**/', [sig_sin_phi, 'g', 'l'],
                                      'rhs'))
        self.add_component(blocks.Signal('out', sig_phi, outport))

class PID(blocks.Submodel):
    def __init__(self, Kp, Ki, Kd, x0, inport, outport):
        blocks.Submodel.__init__(self, 'PID', [inport], [outport])

        sig_xKp  = self.make_signal_name('xKp')
        sig_ix   = self.make_signal_name('ix')
        sig_ixKi = self.make_signal_name('ixKi')
        sig_dx   = self.make_signal_name('dx')
        sig_dxKd = self.make_signal_name('dxKd')

        self.add_component(blocks.Gain(Kp, 'Kp', inport, sig_xKp))
        self.add_component(blocks.Integrator(x0, 'ix', inport, sig_ix))
        self.add_component(blocks.Gain(Ki, 'Ki', sig_ix, sig_ixKi))
        self.add_component(blocks.Derivative(x0, 'dx', inport, sig_dx))
        self.add_component(blocks.Gain(Kd, 'Kd', sig_dx, sig_dxKd))
        self.add_component(blocks.AddSub('', '+++', [sig_xKp, sig_ixKi, sig_dxKd], outport))

class SSModel(blocks.MainModel):
    def __init__(self):
        blocks.MainModel.__init__(self, 'pendulum_with_pid')

        sig_phi = 'phi'
        sig_tau = 'tau'
        sig_err = 'err'

        self.add_component(blocks.AddSub('', '+-', ['des_phi', sig_phi], sig_err))
        self.add_component(PID(40.0, 20.0, 0.05, 0.0, sig_err, sig_tau))
        self.add_component(Pendulum(sig_tau, sig_phi))

class DrawPendulum(pyglet.window.Window):

    def __init__(self, width, height, history):
        super().__init__(width, height, "Shapes")
        self.time = 0
        self.batch = pyglet.graphics.Batch()
        self.history = history

        self.square = shapes.Rectangle(width//2, height//2, 10, 100, color=(55, 55, 255),
                                               batch=self.batch)
        self.square.anchor_position = 5, 0

    def on_draw(self):
        """Clear the screen and draw shapes"""
        self.clear()
        self.batch.draw()

    def update(self, delta_time):
        """Animate the shapes"""
        self.time += delta_time
        
        ind = bisect.bisect_left(self.history['t'], self.time)
        if ind < len(self.history['t']):
            phi = (180/np.pi)*self.history['pendulum.phi'][ind] + 180
            print(ind, phi)
            self.square.rotation = phi
        
def main():
    model = SSModel()
    states = []
    model.get_states(states)

    parameters = {
        'm': 3.0,
        'l': 0.1,
        'g': 9.81,
        'des_phi': np.pi/6,
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
    while t < 10.0:
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
    
    vis = DrawPendulum(720, 480, history)
    pyglet.clock.schedule_interval(vis.update, 1/30)
    pyglet.app.run()
    
if __name__ == '__main__':
    main()
