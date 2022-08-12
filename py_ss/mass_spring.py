#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Created on Wed Aug 10 20:52:40 2022

@author: fathi
"""

class Base:
    all_inports_ = []
    all_outports_ = []
    
    def __init__(self, name, inports=[], outports=[]):
        self.name_ = name
        self.inports_ = inports
        self.outports_ = outports
        
        for port in outports:
            assert port not in self.all_outports

        for port in inports:
            if port not in self.all_inports:
                self.all_inports.append(port)

        self.all_outports_ += outports

    def get_states(self, states):
        return

class Signal:
    def __name__(self, src, dst, name):
        self.src_ = src
        self.dst_ = dst
        self.name_ = name

class Gain(Base):
    def __init__(self, k, name):
        Base.__init__(self, name)
        self.k_ = k

class Integrator(Base):
    def __init__(self, t0, x0, name):
        Base.__init__(self, name)
        self.time_ = t0
        self.value_ = x0
    
    def get_states(self, states):
        states[self.name_] = self.value_

class Submodel(Base):
    def __init__(self, name):
        Base.__init__(self, name)
        self.submodels_ = []

    def add_submodel(self, model):
        model.name_ = self.name_ + '.' + model.name_
        self.submodels_.append(model)

    def get_states(self, states):
        for m in self.submodels_:
            m.get_states(states)

class SSModel(Submodel):
    def __init__(self):
        Submodel.__init__(self, 'mass_spring')

        self.xd_ = Integrator(0.0, 0.1, 'xd')
        self.x_  = Integrator(0.0, 0.0, 'x')
        
        self.add_submodel(self.xd_)
        self.add_submodel(self.x_)
    
    def __call__(self, t, x):
        # create the dictionary
        x = {k: v for k, v in zip(self.state_names_, x)}

def rk4(model, t0, x0, h):
    k1 = h*model(t0, x0)
    k2 = h*model(t0 + h/2, x0 + k1/2)
    k3 = h*model(t0 + h/2, x0 + k2/2)
    k4 = h*model(t0 + h, x0 + k3)
    return x0 + k1/6 + k2/3 + k3/3 + k4/6

if __name__ == '__main__':
    model = SSModel()
    states = {}
    model.get_states(states)
    print(states)
