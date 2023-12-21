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

#define CALC_K(K, t, v) \
    _X.set_states(v); \
    callback(t, _X); \
    K = h * _X.derivs();

Euler::Euler(const Model& model) :
    StepperBase(model), _X(model)
{
}

void Euler::step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h)
{
    double h = new_h = t1 - t0;

    // K = h * f(t0, v0)
    CALC_K(_K, t0, v0)

    // v1 = v0 + K
    v1 = v0 + _K;
}

Rk4::Rk4(const Model& model) :
    StepperBase(model), _X(model)
{
}

void Rk4::step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h)
{
    double h = new_h = t1 - t0;

    // K1 = h * f(t0, v0)
    CALC_K(_K1, t0, v0)

    // K2 = h * f(t0 + h/2, v0 + K1/2)
    CALC_K(_K2, t0 + h/2, v0 + _K1 / 2)

    // K3 = h * f(t0 + h/2, v0 + K2/2)
    CALC_K(_K3, t0 + h/2, v0 + _K2 / 2)

    // K4 = h * f(t0 + h, v0 + K3)
    CALC_K(_K4, t0 + h, v0 + _K3)

    // v1 = v0 + K1/6 + K2/3 + K3/3 + K4/6
    v1 = v0 + _K1 / 6 + _K2 / 3 + _K3 / 3 + _K4 / 6;
}

Rkf45::Rkf45(const Model& model) :
    StepperBase(model), _X(model)
{
}

void Rkf45::step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h)
{
    static Array YT(v0.size());
    static Array ZT(v0.size());

    double h = t1 - t0;
    static const double eps_abs = 0.001;

    // K1 = h * f(t0, v0)
    CALC_K(_K1, t0, v0)

    // K2 = h * f(t0 + h/4, v0 + K1/4)
    CALC_K(_K2, t0 + h/4, v0 + 1./4 * _K1)

    // K3 = h * f(t0 + 3/8 * h, v0 + 3/8 * ( 1/4 * K1 + 3/4 * K2))
    CALC_K(_K3, t0 + 3./8 * h, v0 + (3./8 * 1./4) * _K1 + (3./8 * 3./4) * _K2)

    // K4 = h * f(t0 + 12/13 * h, v0 + 12/13 * (161/169 * K1 - 600/169 * K2 + 608/169 * K3))
    CALC_K(_K4, t0 + 12./13 * h, v0 + (12./13 * 161./169) * _K1 - (12./13 * 600./169) * _K2 + (12./13 * 608./169) * _K3)

    // K5 = h * f(t0 + h, v0 + 8341/4104 * K1 - 32832/4104 * K2 + 29440/4104 * K3 - 845/4104 * K4)
    CALC_K(_K5, t1, v0 + (8341./4104) * _K1 - (32832./4104) * _K2 + (29440./4104) * _K3 - (845./4104) * _K4)

    // K6 = h * f(t0 + 1/2 * h, v0 + 1/2 * (-6080/10260 * K1 + 41040/10260 * K2 - 28352/10260 * K3 + 9295/10260 * K4 - 5643/10260 * K5))
    CALC_K(_K6, t0 + 1./2 * h, v0 - (1./2 * 6080./10260) * _K1 + (1./2 * 41040./10260) * _K2 - (1./2 * 28352./10260) * _K3 + (1./2 * 9295./10260) * _K4 - (1./2 * 5643./10260) * _K5)

    // yt := y[i - 1] + h*(  2375*K[1] +  11264*K[3] +  10985*K[4] - 4104*K[5] )/20520;
    v1 = v0 + (2375./20520) * _K1 + (11264./20520) * _K3 + (10985./20520) * _K4 - (4104./20520) * _K5;

    // zt := y[i - 1] + h*( 33440*K[1] + 146432*K[3] + 142805*K[4] - 50787*K[5] + 10260*K[6] )/282150;
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
