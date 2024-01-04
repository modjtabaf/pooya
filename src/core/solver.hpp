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

#ifndef __POOYA_SOLVER_HPP__
#define __POOYA_SOLVER_HPP__

#include "signal.hpp"

namespace pooya
{

class StepperBase
{
public:
    using StepperCallback = std::function<void(double, Values&)>;

protected:
    const Model& _model;
    Values _X;
    StepperCallback _callback;

    auto f(double t, const Array& v) -> const auto&
    {
        _X.set_states(v);
        _callback(t, _X);
        return _X.derivs();
    }

public:
    StepperBase(const Model& model) : _model(model), _X(model) {}

    virtual void step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h) = 0;
};

class Euler : public StepperBase
{
public:
    Euler(const Model& model) : StepperBase(model) {}

    void step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h) override;
};

class Rk4 : public StepperBase
{
protected:
    Array _K1;
    Array _K2;
    Array _K3;
    Array _K4;

public:
    Rk4(const Model& model) : StepperBase(model) {}

    void step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h) override;
};

class Rkf45 : public StepperBase
{
protected:
    Array _K1;
    Array _K2;
    Array _K3;
    Array _K4;
    Array _K5;
    Array _K6;
    Array _ZT;

public:
    Rkf45(const Model& model) : StepperBase(model) {}

    void step(StepperCallback callback, double t0, const Array& v0, double t1, Array& v1, double& new_h) override;
};

}

#endif // __POOYA_SOLVER_HPP__
