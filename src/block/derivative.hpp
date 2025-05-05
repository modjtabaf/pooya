/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_BLOCK_DERIVATIVE_HPP__
#define __POOYA_BLOCK_DERIVATIVE_HPP__

#include "singleio.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class DerivativeT : public SingleInputOutputT<T>
{
public:
    using Base = SingleInputOutputT<T>;

protected:
    bool _first_step{true};
    double _t;
    T _x;
    T _y;

public:
    explicit DerivativeT(const T& y0 = 0) : Base(1), _y(y0) {}
    DerivativeT(Submodel* parent, std::string_view name = "", const T& y0 = 0) : Base(parent, name, 1), _y(y0) {}

#if __cplusplus >= 202002L // C++20
    struct Params
    {
        Submodel* parent{nullptr};
        std::string_view name{""};
        T initial{0};
    };
    static_assert(std::is_aggregate_v<Params>);

    explicit DerivativeT(const Params& params)
        : Base({.parent = params.parent, .name = params.name}), _y(params.initial)
    {
    }
#endif // __cplusplus >= 202002L // C++20

    void post_step(double t) override
    {
        pooya_trace("block: " + Base::full_name().str());
        _t          = t;
        _x          = *Base::_s_in;
        _y          = _x;
        _first_step = false;
    }

    void activation_function(double t) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (_first_step)
        {
            _t = t;
            _x = *Base::_s_in;
            Base::_s_out->set(_y);
        }
        else if (_t == t)
        {
            Base::_s_out->set(_y);
        }
        else
        {
            Base::_s_out->set((*Base::_s_in - _x) / (t - _t));
        }
    }
};

using Derivative  = DerivativeT<double>;
using DerivativeA = DerivativeT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_DERIVATIVE_HPP__
