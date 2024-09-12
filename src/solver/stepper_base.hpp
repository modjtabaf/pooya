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

#ifndef __POOYA_SOLVER_STEPPER_BASE_HPP__
#define __POOYA_SOLVER_STEPPER_BASE_HPP__

#include <functional>

#include "src/helper/trace.hpp"
#include "src/signal/array.hpp"
#include "src/signal/values_array.hpp"

namespace pooya
{

class StepperBase
{
public:
    using StepperCallback = std::function<const ValuesArray::StateVariableDerivs&(double, const Array&)>;

protected:
    StepperCallback _callback;

    auto f(double t, const Array& v) -> const ValuesArray::StateVariableDerivs&
    {
        pooya_trace0;
        return _callback(t, v);
    }

public:
    StepperBase() = default;

    virtual void step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h) = 0;
};

}

#endif // __POOYA_SOLVER_STEPPER_BASE_HPP__
