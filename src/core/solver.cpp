/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <ostream>
#include <type_traits>
#include "solver.hpp"

namespace pooya
{

void Euler::step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h)
{
    _callback = callback;
    double h = new_h = t1 - t0;

    v1 = v0 + h * f(t0, v0);
}

void Rk4::step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h)
{
    _callback = callback;
    double h = new_h = t1 - t0;

    _K1 = h * f(t0, v0);
    _K2 = h * f(t0 + h/2, v0 + _K1 / 2);
    _K3 = h * f(t0 + h/2, v0 + _K2 / 2);
    _K4 = h * f(t0 + h  , v0 + _K3);
    v1 = v0 + _K1 / 6 + _K2 / 3 + _K3 / 3 + _K4 / 6;
}

void Rkf45::step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h)
{
    _callback = callback;
    double h = t1 - t0;
    static const double eps_abs = 0.001;

    _K1 = h * f(t0, v0);
    _K2 = h * f(t0 +  1./4  * h, v0 + 1./4 * _K1);
    _K3 = h * f(t0 +  3./8  * h, v0 + (3./8 * 1./4) * _K1 + (3./8 * 3./4) * _K2);
    _K4 = h * f(t0 + 12./13 * h, v0 + (12./13 * 161./169) * _K1 - (12./13 * 600./169) * _K2 + (12./13 * 608./169) * _K3);
    _K5 = h * f(t1             , v0 + (8341./4104) * _K1 - (32832./4104) * _K2 + (29440./4104) * _K3 - (845./4104) * _K4);
    _K6 = h * f(t0 +  1./2  * h, v0 - (1./2 * 6080./10260) * _K1 + (1./2 * 41040./10260) * _K2 - (1./2 * 28352./10260) * _K3 + (1./2 * 9295./10260) * _K4 - (1./2 * 5643./10260) * _K5);

    v1  = v0 + (2375./20520) * _K1 + (11264./20520) * _K3 + (10985./20520) * _K4 - (4104./20520) * _K5;
    _ZT = v0 + (33440./282150) * _K1 + (146432./282150) * _K3 + (142805./282150) * _K4 - (50787./282150) * _K5 + (10260./282150) * _K6;

    double max_abs = (v1 - _ZT).abs().max(0)(0, 0);
    double s4 = max_abs > 0 ? (eps_abs * h) / (2 * max_abs) : 1;

    if (s4 < 0.31640625) // 0.31640625 == 0.75 ^ 4
        new_h = h / 2;
    else if (s4 > 5.0625) // 5.0625 == 1.5 ^ 4
        new_h = 2 * h;
    else
        new_h = h;
}

}
