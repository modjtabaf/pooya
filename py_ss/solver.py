#!/usr/bin/env python3
# -*- coding: utf-8 -*-

def rk4(callback, t0, x0, h):
    k1 = h*callback(t0, x0)
    k2 = h*callback(t0 + h/2, x0 + k1/2)
    k3 = h*callback(t0 + h/2, x0 + k2/2)
    k4 = h*callback(t0 + h, x0 + k3)
    return x0 + k1/6 + k2/3 + k3/3 + k4/6

def simple(callback, t0, x0, h):
    return x0 + h*callback(t0, x0)
