#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import string
import random

class State:
    def __init__(self, state, deriv, value):
        self._state = state
        self._deriv = deriv
        self._value = value
    
    def __repr__(self):
        return self._state + ", " + self._deriv + ', ' + str(self._value)

class Node(str):
    pass

def N(s):
    if isinstance(s, Node):
        return s
    if isinstance(s, list):
        return [N(a) for a in s]
    return Node(s)

class Base:
    _all_iports = []
    _all_oports = []
    
    def __init__(self, name, iports=[], oports=[], **kwargs):
        if not isinstance(iports, (list, tuple)):
            iports = [iports]
        if not isinstance(oports, (list, tuple)):
            oports = [oports]

        parent = Submodel.current()

        self._name = name
        self._processed = False
        
        if parent:
            if parent._name:
                self._name = parent._name + '.' + self._name
            parent.add_component(self)

            iports = [parent.auto_signal_name(False) if p == '-'
                      else parent.make_signal_name(p) for p in iports]
            oports = [parent.auto_signal_name(True) if p == '-'
                      else parent.make_signal_name(p) for p in oports]

        self._iports = iports
        self._oports = oports

        if kwargs.get('register_oports', True):
            for port in oports:
                assert port not in self._all_oports, 'Port ' + port \
                    + ' already exists in oports: {}'.format(self._all_oports)
                self._all_oports.append(port)

        for port in iports:
            if port not in self._all_iports:
                self._all_iports.append(port)

    def get_states(self, states):
        return

    def step(self, t, states):
        return

    def get_ports(self):
        return self._iports, self._oports

    @property
    def is_processed(self): return self._processed

    def _process(self, t, x, reset):
        if reset:
            self._processed = False

        if self._processed:
            return 0
        else:
            for iport in self._iports:
                if iport not in x:
                    return 0

            self._known_values = x

            for oport in self._oports:
                assert oport not in x, self._name + ': oport ' \
                    + oport + ' already exists in oports'

            input_values = [x[iport] for iport in self._iports]
            output_values = self.activation_function(t, input_values)
            for oport, output_value in zip(self._oports, output_values):
                x[oport] = output_value

            self._processed = True
            return 1

    def __repr__(self):
        return str(type(self)) + ":" + self._name + ", iports:" + str(self._iports) + ", oports:" + str(self._oports)

    def traverse(self, cb):
        return cb(self)

class Signal(Base):
    def activation_function(self, t, x):
        return [x[0]]

class Bus(Base):
    class BusValues(list):
        def __init__(self, names, *args):
            super().__init__(args)
            self._names = names
        
        def __repr__(self):
            return 'BusValues(' + super().__repr__() + ')'

    def __init__(self, name, iports='-', oport='-'):
        super().__init__(name, iports, oport)
        
        if not isinstance(iports, (list, tuple)):
            iports = [iports]
        self._raw_names = [n for n in iports]

    def activation_function(self, t, x):
        ret = [Bus.BusValues(self._raw_names, *x)]
        return ret

class BusSelector(Base):
    def __init__(self, name, iport='-', signals=[], oports=[]):
        if not isinstance(oports, (list, tuple)):
            oports = [oports]
        if not isinstance(signals, (list, tuple)):
            signals = [signals]
            
        if not signals:
            signals = oports
        elif not oports:
            oports = signals
        assert len(signals) == len(oports)

        super().__init__(name, iport, oports)
        self._signals = signals
        self._indices = None

    def activation_function(self, t, x):
        bus_values = x[0]
        if self._indices is None:
            self._indices = [bus_values._names.index(s)
                             for s in self._signals]
        return [bus_values[k] for k in self._indices]

class InitialValue(Base):
    def __init__(self, name, iport='-', oport='-'):
        super().__init__(name, iport, oport)
        self._value = None

    def activation_function(self, t, x):
        if self._value is None:
            self._value = x[0]
            self._iports = []
        return [self._value]

class Const(Base):
    def __init__(self, name, value, oport='-'):
        super().__init__(name, iports=[], oports=oport)
        self._value = value

    def activation_function(self, t, x):
        return [self._value]

class Gain(Base):
    def __init__(self, name, k, iport='-', oport='-'):
        super().__init__(name, iports=iport, oports=oport)
        self._k = k

    def activation_function(self, t, x):
        return [self._k * x[0]]

class Function(Base):
    def __init__(self, name, act_func, iport='-', oport='-'):
        super().__init__(name, iports=iport, oports=oport)
        self._act_func = act_func

    def activation_function(self, t, x):
        return [self._act_func(t, x[0])]

class MIMOFunction(Base):
    def __init__(self, name, act_func, iports, oports):
        super().__init__(name, iports=iports, oports=oports)
        self._act_func = act_func

    def activation_function(self, t, x):
        return self._act_func(t, x)

class AddSub(Base):
    def __init__(self, name, operations, iports, oport='-', initial=0.0):
        assert len(operations) == len(iports)
        super().__init__(name, iports=iports, oports=oport)
        self._operations = operations
        self._initial = initial

    def activation_function(self, t, x):
        ret = self._initial
        for operation, v in zip(self._operations, x):
            if operation == '+':
                ret += v
            elif operation == '-':
                ret -= v
        return [ret]

class MulDiv(Base):
    def __init__(self, name, operations, iports, oport='-', initial=1.0):
        assert len(operations) == len(iports)
        super().__init__(name, iports=iports, oports=oport)
        self._operations = operations
        self._initial = initial

    def activation_function(self, t, x):
        ret = self._initial
        for operation, v in zip(self._operations, x):
            if operation == '*':
                ret *= v
            elif operation == '/':
                ret /= v
        return [ret]

class Integrator(Base):
    def __init__(self, name, iport='-', oport='-', x0=0.0):
        super().__init__(name, iports=iport, oports=oport)
        self._value = x0

    def get_states(self, states):
        states.append(State(self._oports[0], self._iports[0], self._value))

    def step(self, t, states):
        self._value = states[self._oports[0]]

    def _process(self, t, x, reset):
        # self._processed = True
        # return 1 if reset else 0
        if reset:
            self._processed = False

        if self._processed:
            return 0
        else:
            self._processed = self._iports[0] in x
            return 1 if self._processed else 0

# it is still unclear how to deal with states when using this numerical integrator
# class NumericalIntegrator(Base):
#     def __init__(self, y0, name, iport='-', oport='-'):
#         super().__init__(name, [iport], [oport])
#         self._t = None
#         self._x = None
#         self._y = y0
    
#     def step(self, t, states):
#         self._t = t
#         self._x = states[self._iports[0]]
#         self._y = states[self._oports[0]]

#     def activation_function(self, t, x):
#         if self._t is None:
#             self._t = t
#             self._x = x[0]
#             return [self._y]
#         else:
#             return [self._y + 0.5*(t - self._t)*(x[0] + self._x)]

class Delay(Base):
    def __init__(self, name, iports, oport='-', lifespan=10.0):
        super().__init__(name, iports=iports, oports=oport)
        self._t = []
        self._x = []
        self._lifespan = lifespan

    def step(self, t, states):
        if self._t:
            t1 = t - self._lifespan
            for k, v in enumerate(self._t):
                if v >= t1:
                    break
            self._t = self._t[k:]
            self._x = self._x[k:]

        assert (not self._t) or (t > self._t[-1])
        self._t.append(t)
        self._x.append(states[self._iports[0]])

    def activation_function(self, t, x):
        if self._t:
            delay = x[1]
            return [np.interp(t - delay, self._t, self._x)]
        return [x[2]]

class Memory(Base):
    def __init__(self, name, iport='-', oport='-', ic=0.0):
        super().__init__(name, iports=iport, oports=oport)
        self._value = ic

    def step(self, t, states):
        self._value = states[self._iports[0]]

    # Memory can be implemented either by defining the following activation function
    #   (which is more straightforward) or through overloading the _process method
    #   which is more efficient since it deosn't rely on the input signal being known.
    #   Both approaches are supposed to lead to the exact same results.

    # def activation_function(self, t, x):
    #     return [self._value]

    def _process(self, t, x, reset):
        if reset:
            self._processed = False

        if self._processed:
            return 0
        else:
            x[self._oports[0]] = self._value
            self._processed = True
            return 1

class Derivative(Base):
    def __init__(self, name, iport='-', oport='-', y0=0):
        super().__init__(name, iports=iport, oports=oport)
        self._t = None
        self._x = None
        self._y = y0

    def step(self, t, states):
        self._t = t
        self._x = states[self._iports[0]]
        self._y = states[self._oports[0]]

    def activation_function(self, t, x):
        if self._t is None:
            self._t = t
            self._x = x[0]
            return [self._y]
        elif self._t == t:
            return [self._y]
        else:
            return [(x[0] - self._x)/(t - self._t)]

class Submodel(Base):
    _current_submodels = []

    def current():
        return Submodel._current_submodels[-1] if len(Submodel._current_submodels) > 0 else None

    def __init__(self, name, iports=[], oports=[]):
        super().__init__(name, iports=iports, oports=oports, register_oports=False)
        self._components = []
        self._auto_signal_name = ''

    def __enter__(self):
        Submodel._current_submodels.append(self)
        return self

    def __exit__(self, type, value, traceback):
        assert Submodel._current_submodels[-1] == self
        del Submodel._current_submodels[-1]
        return True

    def add_component(self, component):
        self._components.append(component)

    def get_states(self, states):
        for component in self._components:
            component.get_states(states)

    def step(self, t, states):
        for component in self._components:
            component.step(t, states)

    def make_signal_name(self, name):
        if isinstance(name, Node):
            return name
        if not isinstance(name, str):
            name = str(name)
        if self._name:
            if name[0] == '-':
                name = '-' + self._name + '.' + name[1:]
            else:
                name = self._name + '.' + name
        return Node(name)

    def auto_signal_name(self, makenew):
        if makenew:
            letters = string.ascii_letters
            self._auto_signal_name = '-' + (''.join(random.choice(letters)
                                              for i in range(10)))
        else:
            assert self._auto_signal_name
        return self._auto_signal_name

    def _process(self, t, x, reset):
        if reset:
            self._processed = False

        self._known_values = x

        n_processed = 0
        if not self._processed:
            self._processed = True
            for component in self._components:
                n_processed += component._process(t, x, reset)
                if not component.is_processed:
                    self._processed = False

        return n_processed

    def traverse(self, cb):
        for component in self._components:
            if not component.traverse(cb):
                return False
        return super().traverse(cb)
