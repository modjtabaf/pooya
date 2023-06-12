#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import scipy.io as spio
from scipy.interpolate import interp1d
import os

import blocks, solver

def run(model, T, inputs_cb=lambda t, x: {}, parameters={}, stepper=solver.rk4):
    def process(t, x):
        n_processed = model._process(t, x, True)
        while True:
            n = model._process(t, x, False)
            if n == 0:
                break
            n_processed += n

        unprocessed = []
        def find_unprocessed_cb(c):
            if not c.is_processed:
                unprocessed.append(c)
            return True
        model.traverse(find_unprocessed_cb)
        if unprocessed:
            print('-- unprocessed blocks detected:')
            for c in unprocessed:
                print('-', c._name)
                for p in c._iports:
                    print('  - i: ', '*' if p not in x else ' ', p)
                for p in c._oports:
                    print('  - o: ', '*' if p not in x else ' ', p)

        return n_processed

    history = {}

    states = []
    model.get_states(states)

    def stepper_callback(t, x):
        y = {}
        
        k = 0
        for state in states:
            if isinstance(state._value, np.ndarray):
                k2 = k + state._value.size
                y[state._state] = x[k:k2].reshape(state._value.shape)
                k = k2
            else:
                y[state._state] = x[k]
                k += 1

        for p, v in parameters.items():
            y[p] = v

        for p, v in inputs.items():
            y[p] = v

        process(t, y)

        ret = []
        for state in states:
            if isinstance(state._value, np.ndarray):
                ret.extend(y[state._deriv].ravel())
            else:
                ret.append(y[state._deriv])
        ret = np.array(ret)
        
        return ret

    x = []
    for state in states:
        if isinstance(state._value, np.ndarray):
            for y in state._value.ravel():
                x.append(y)
        else:
            x.append(state._value)
    x = np.array(x)

    def update_history(t, x):
        y = {}
        
        k = 0
        for state in states:
            if isinstance(state._value, np.ndarray):
                k2 = k + state._value.size
                y[state._state] = x[k:k2].reshape(state._value.shape)
                k = k2
            else:
                y[state._state] = x[k]
                k += 1

        for p, v in parameters.items():
            y[p] = v

        for p, v in inputs.items():
            y[p] = v

        process(t, y)

        model.step(t, y)
    
        if not history:
            history['t'] = []
            for k, v in y.items():
                if k not in parameters and k[0] != '-' \
                    and isinstance(v, (int, float, np.ndarray)):
                    history[k] = []
    
        history['t'].append(t)
        for k, v in y.items():
            if k not in parameters and k[0] != '-' \
                    and isinstance(v, (int, float, np.ndarray)):
                    history[k].append(v)

    if states:
        t = T[0]
        k = 0
        for t1 in T[1:]:
            if k%100 == 0:
                print('%d: %.3f' %(k, t))
            inputs = inputs_cb(t, x)
            update_history(t, x)
            x = stepper(stepper_callback, t0=t, t1=t1, x0=x)
            t = t1
            k += 1
        inputs = inputs_cb(t, x)
        update_history(t, x)
    else:
        for t in T:
            inputs = inputs_cb(t, x)
            x = stepper_callback(t, x)
            update_history(t, x)

    history = {k: np.array(v) for k, v in history.items()}
    return history

def load_mat_files_as_bus(root, prefix):
    prefix += '.'
    ret = {}
    dirpath, dirnames, filenames = next(os.walk(root))
    for filename in filenames:
        if not (filename.startswith(prefix) and filename.endswith('.mat')):
            continue
        parts = filename.split('.')
        cur = ret
        for part in parts[1:-2]:
            cur = cur.setdefault(part, {})
        foo = spio.loadmat(os.path.join(dirpath, filename))
        foo = foo['data']
        if foo.shape[0] == 1:
            cur[parts[-2]] = foo[:, 1:].squeeze()
        else:
            cur[parts[-2]] = interp1d(
                foo[:, 0], foo[:, 1:].squeeze(), axis=0, assume_sorted=True,
                fill_value='extrapolate')

    return ret

def interp_bus(dct, t):
    def traverse(d, r):
        for k, v in d.items():
            if isinstance(v, dict):
                r._names.append(k)
                r.append(blocks.Bus.BusValues([]))
                traverse(v, r[-1])
            elif isinstance(v, interp1d):
                foo = v(t)
                r._names.append(k)
                r.append(foo if foo.shape else float(foo))
            elif isinstance(v, np.ndarray):
                r._names.append(k)
                r.append(v if v.shape else float(v))

    ret = blocks.Bus.BusValues([])
    traverse(dct, ret)
    return ret
