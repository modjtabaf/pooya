#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import bisect
import pyglet
from pyglet import shapes

import blocks, helper
from blocks import N as N

class Pendulum(blocks.Submodel):
    def __init__(self, iport, oport):
        super().__init__('pendulum', [iport], [oport])

        # nodes
        tau  = N(iport)
        dphi = N('dphi')
        phi  = N(oport)
        m = N('m')
        l = N('l')
        g = N('g')

        # blocks
        with self:
            blocks.MulDiv('tau/ml2', operations='*///', iports=[tau, m, l, l])
            blocks.AddSub('err', operations='+-', iports=['-', -1])
            blocks.Integrator('dphi', oport=dphi)
            blocks.Integrator('phi', iport=dphi, oport=phi)
            blocks.Function('sin(phi)', act_func=lambda t, x: np.sin(x), iport=phi)
            blocks.MulDiv('g/l', operations='**/', iports=['-', g, l], oport=-1)

class PID(blocks.Submodel):
    def __init__(self, Kp, Ki, Kd, iport, oport, x0=0.0):
        super().__init__('PID', [iport], [oport])

        # nodes
        x = N(iport)

        # blocks
        with self:
            blocks.Gain('Kp', k=Kp, iport=x, oport=-1)
            blocks.Integrator('ix', iport=x, x0=x0)
            blocks.Gain('Ki', k=Ki, oport=-2)
            blocks.Gain('Kd', k=Kd, iport=x)
            blocks.Derivative('dx', oport=-3)
            blocks.AddSub('', operations='+++', iports=[-1, -2, -3],
                          oport=N(oport))

class SSModel(blocks.Submodel):
    def __init__(self):
        super().__init__('')

        # nodes
        phi = N('phi')
        tau = N('tau')
        err = N('err')

        # blocks
        with self:
            blocks.AddSub('',
                          operations='+-',
                          iports=[N('des_phi'), phi],
                          oport=err)
            PID(Kp=40.0, Ki=20.0, Kd=0.05, iport=err, oport=tau)
            Pendulum(iport=tau, oport=phi)

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
        self.time += 0.1*delta_time
        
        ind = bisect.bisect_left(self.history['t'], self.time)
        if ind < len(self.history['t']):
            phi = (180/np.pi)*self.history['phi'][ind] + 180
            print(ind, phi)
            self.square.rotation = phi

def main():
    parameters = {
        'm': 0.2,
        'l': 0.1,
        'g': 9.81,
        'des_phi': np.pi/4,
        }

    model = SSModel()
    history = helper.run(model, T=np.arange(0.0, 5.0, 0.01),
                         parameters=parameters)

    T = history['t']
    plt.figure()
    plt.subplot(3, 1, 1); plt.plot(T, np.rad2deg(history['phi']))
    plt.ylabel('phi')
    plt.subplot(3, 1, 2); plt.plot(T, history['dphi'])
    plt.ylabel('dphi')
    plt.subplot(3, 1, 3); plt.plot(T, history['tau'])
    plt.xlabel('t'); plt.ylabel('tau')
    plt.show()

    vis = DrawPendulum(720, 480, history)
    pyglet.clock.schedule_interval(vis.update, 1/30)
    pyglet.app.run()
    
if __name__ == '__main__':
    main()
