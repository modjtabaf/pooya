#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 12 11:41:22 2022

@author: fathi
"""

import numpy as np

class State:
    def __init__(self, state, deriv, value):
        self._state = state
        self._deriv = deriv
        self._value = value

class Base:
    _all_inports = []
    _all_outports = []
    
    def __init__(self, name, inports=[], outports=[]):
        self._name = name
        self._processed = False
        self._inports = inports
        self._outports = outports

        for port in outports:
            assert port not in self._all_outports, 'Port ' + port + ' already exists it outports: {}'.format(self._all_outports)
            self._all_outports.append(port)

        for port in inports:
            if port not in self._all_inports:
                self._all_inports.append(port)

    def get_states(self, states):
        return

    def update_states(self, t, states):
        return

    def get_ports(self):
        return self._inports, self._outports

    @property
    def is_processed(self): return self._processed

    def _process(self, t, x, reset):
        if reset:
            self._processed = False

        if self._processed:
            return 0
        else:
            for inport in self._inports:
                if inport not in x:
                    return 0

            for outport in self._outports:
                assert outport not in x

            input_values = [x[inport] for inport in self._inports]
            output_values = self.activation_function(t, input_values)
            for outport, output_value in zip(self._outports, output_values):
                x[outport] = output_value

            self._processed = True
            return 1

class Gain(Base):
    def __init__(self, k, name, inport, outport):
        Base.__init__(self, name, [inport], [outport])
        self._k = k

    def activation_function(self, t, x):
        return [self._k * x[0]]

class Sin(Base):
    def __init__(self, name, inport, outport):
        Base.__init__(self, name, [inport], [outport])

    def activation_function(self, t, x):
        return [np.sin(x[0])]

class Integrator(Base):
    def __init__(self, x0, name, inport, outport):
        Base.__init__(self, name, [inport], [outport])
        self._value = x0  # needed?
    
    def get_states(self, states):
        states.append(State(self._outports[0], self._inports[0], self._value))

    def update_states(self, t, states):
        self._value = states[self._outports[0]]

    def _process(self, t, x, reset):
        self._processed = True
        return 1 if reset else 0

class Submodel(Base):
    def __init__(self, name, inports=[], outports=[]):
        Base.__init__(self, name, inports, outports)
        self._components = []

    def add_component(self, component):
        component._name = self._name + '.' + component._name
        self._components.append(component)

    def get_states(self, states):
        for component in self._components:
            component.get_states(states)

    def update_states(self, t, states):
        for component in self._components:
            component.update_states(t, states)

    def _process(self, t, x, reset):
        if reset:
            self._processed = False

        n_processed = 0
        if not self._processed:
            self._processed = True
            for component in self._components:
                n_processed += component._process(t, x, reset)
                if not component.is_processed:
                    self._processed = False

        return n_processed

class MainModel(Submodel):
    def process(self, t, x):
        n_processed = self._process(t, x, True)
        while True:
            n = self._process(t, x, False)
            if n == 0:
                break
            n_processed += n

        return n_processed
