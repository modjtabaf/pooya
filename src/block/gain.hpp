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

#ifndef __POOYA_BLOCK_GAIN_HPP__
#define __POOYA_BLOCK_GAIN_HPP__

#include "singleio.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T, typename GainType>
class GainT : public SingleInputOutputT<T>
{
public:
    using Base = SingleInputOutputT<T>;

protected:
    GainType _k;

public:
    GainT(const GainType& k) : Base(1), _k(k) {}
    GainT(Submodel* parent, std::string_view name, const GainType& k) : Base(parent, name, 1), _k(k) {}

#if __cplusplus >= 202002L // C++20
    explicit GainT(const GainType& k, const Base::Params& params) : Base(params), _k(k) {}
#endif // __cplusplus >= 202002L // C++20

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + Base::full_name().str());
        Base::_s_out->set(_k * Base::_s_in->get());
    }

    typename Types<GainType>::GetValue gain() const { return _k; }
};

using Gain  = GainT<double, double>;
using GainI = GainT<int, int>;
using GainA = GainT<Array, double>;

} // namespace pooya

#endif // __POOYA_BLOCK_GAIN_HPP__
