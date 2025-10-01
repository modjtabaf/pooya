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

#ifndef __POOYA_BLOCK_TRIGGERED_INTEGRATOR_HPP__
#define __POOYA_BLOCK_TRIGGERED_INTEGRATOR_HPP__

#include "src/block/integrator_base.hpp"
#include "src/signal/array.hpp"
#include "src/signal/bool_signal.hpp"

namespace pooya
{

template<typename T>
class TriggeredIntegratorT : public IntegratorBaseT<T>
{
public:
    using Base = IntegratorBaseT<T>;

    explicit TriggeredIntegratorT(T ic = T(0.0)) : Base(ic, 2, 1) {}
    explicit TriggeredIntegratorT(Submodel* parent, T ic = T(0.0)) : Base(parent, ic, 2, 1) {}

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (!Base::connect(ibus, obus))
        {
            return false;
        }

        _trigger.reset(Base::input("trigger"));

        return true;
    }

    void pre_step(double t) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (_triggered)
        {
            if constexpr (std::is_same_v<T, Array>)
            {
                Base::_value.setZero();
            }
            else
            {
                Base::_value = 0;
            }
            _triggered = false;
        }
        Base::pre_step(t);
    }

    uint _process(double t, bool go_deep = true) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (Base::_processed || !_trigger->is_assigned())
        {
            return 0;
        }

        if (!_triggered)
        {
            _triggered = _trigger;
        }

        return Base::_process(t, go_deep);
    }

protected:
    BoolSignal _trigger;
    bool _triggered{false};
};

using TriggeredIntegrator = TriggeredIntegratorT<double>;

#ifdef POOYA_ARRAY_SIGNAL
using TriggeredIntegratorA = TriggeredIntegratorT<Array>;
#endif // POOYA_ARRAY_SIGNAL

} // namespace pooya

#endif // __POOYA_BLOCK_TRIGGERED_INTEGRATOR_HPP__
