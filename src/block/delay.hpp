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

#include "src/signal/array.hpp"
#include "src/signal/scalar_signal.hpp"
#include "singleio.hpp"

namespace pooya
{

template <typename T>
class DelayT : public SingleOutputT<T>
{
protected:
    double _lifespan;
    std::vector<double> _t;
    std::vector<T> _x;

    // input signals
    typename Types<T>::SignalId _s_x;       // in
    ScalarSignalId _s_delay;                // delay
    typename Types<T>::SignalId _s_initial; // initial

public:
    explicit DelayT(double lifespan)
        : SingleOutputT<T>(3, 1), _lifespan(lifespan) {}
    DelayT(const ValidName& name, double lifespan)
        : SingleOutputT<T>(name, 3, 1), _lifespan(lifespan) {}

    bool init(Submodel* parent, const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (!SingleOutputT<T>::init(parent, ibus, obus)) {return false;}

        // input signals
        _s_x = std::move(Types<T>::as_signal_id(SingleOutputT<T>::_ibus.at("in")));
        _s_delay = SingleOutputT<T>::scalar_input_at("delay");
        _s_initial = std::move(Types<T>::as_signal_id(SingleOutputT<T>::_ibus.at("initial")));

        return true;
    }

    void post_step(double t) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (!_t.empty())
        {
            double t1 = t - _lifespan;
            int k = 0;
            for (const auto &v : _t)
            {
                if (v >= t1) {break;}
                k++;
            }
            _t.erase(_t.begin(), _t.begin() + k);
            _x.erase(_x.begin(), _x.begin() + k);
        }

        assert(_t.empty() || (t > _t.back()));
        _t.push_back(t);
        _x.push_back(_s_x->get());
    }

    void activation_function(double t) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (_t.empty())
        {
            SingleOutputT<T>::_s_out->set(_s_initial->get());
            return;
        }

        t -= _s_delay->get();
        if (t <= _t[0])
        {
            SingleOutputT<T>::_s_out->set(_s_initial->get());
        }
        else if (t >= _t.back())
        {
            SingleOutputT<T>::_s_out->set(_x.back());
        }
        else
        {
            int k = 0;
            for (const auto &v : _t)
            {
                if (v >= t) {break;}
                k++;
            }

            SingleOutputT<T>::_s_out->set((_x[k] - _x[k - 1]) * (t - _t[k - 1]) / (_t[k] - _t[k - 1]) + _x[k - 1]);
        }
    }
};

using Delay = DelayT<double>;
using DelayA = DelayT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_DELAY_HPP__
