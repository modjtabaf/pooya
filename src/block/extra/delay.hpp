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

#ifndef __POOYA_BLOCK_DELAY_HPP__
#define __POOYA_BLOCK_DELAY_HPP__

#include "src/block/singleio.hpp"
#include "src/signal/array.hpp"
#include "src/signal/scalar_signal.hpp"

namespace pooya
{

template<typename T>
class DelayT : public SingleOutputT<T>
{
public:
    using Base = SingleOutputT<T>;

    explicit DelayT(double lifespan, Submodel* parent = nullptr, std::string_view name = "")
        : Base(parent, name, 3, 1), _lifespan(lifespan)
    {
    }

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (!Base::connect(ibus, obus))
        {
            return false;
        }

        // input signals
        _s_x.reset(Base::input("in"));
        _s_delay.reset(Base::input("delay"));
        _s_initial.reset(Base::input("initial"));

        return true;
    }

    void post_step(double t) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (!_t.empty())
        {
            double t1 = t - _lifespan;
            int k     = 0;
            for (const auto& v : _t)
            {
                if (v >= t1)
                {
                    break;
                }
                k++;
            }
            _t.erase(_t.begin(), _t.begin() + k);
            _x.erase(_x.begin(), _x.begin() + k);
        }

        pooya_debug_verify0(_t.empty() || (t > _t.back()));
        _t.push_back(t);
        _x.push_back(_s_x);
    }

    void activation_function(double t) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (_t.empty())
        {
            Base::_s_out = _s_initial;
            return;
        }

        t -= _s_delay;
        if (t <= _t[0])
        {
            Base::_s_out = _s_initial;
        }
        else if (t >= _t.back())
        {
            Base::_s_out = _x.back();
        }
        else
        {
            int k = 0;
            for (const auto& v : _t)
            {
                if (v >= t)
                {
                    break;
                }
                k++;
            }

            Base::_s_out = (_x[k] - _x[k - 1]) * (t - _t[k - 1]) / (_t[k] - _t[k - 1]) + _x[k - 1];
        }
    }

protected:
    double _lifespan;
    std::vector<double> _t;
    std::vector<T> _x;

    // input signals
    typename Types<T>::Signal _s_x;       // in
    ScalarSignal _s_delay;                // delay
    typename Types<T>::Signal _s_initial; // initial
};

using Delay = DelayT<double>;

#ifdef POOYA_ARRAY_SIGNAL
using DelayA = DelayT<Array>;
#endif // POOYA_ARRAY_SIGNAL

} // namespace pooya

#endif // __POOYA_BLOCK_DELAY_HPP__
