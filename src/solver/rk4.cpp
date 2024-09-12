/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "src/helper/trace.hpp"
#include "rk4.hpp"

namespace pooya
{

void Rk4::step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h)
{
    pooya_trace0;
    _callback = callback;
    double h = new_h = t1 - t0;
    double h_2 = h / 2;

    _K1 = h_2 * f(t0      , v0      );
    _K2 = h_2 * f(t0 + h_2, v0 + _K1);
    _K3 = h   * f(t0 + h_2, v0 + _K2);
    _K4 = h   * f(t0 + h  , v0 + _K3);

    v1 = v0 + (1./3) * _K1 + (2./3) * _K2 + (1./3) * _K3 + (1./6) * _K4;
}

}
