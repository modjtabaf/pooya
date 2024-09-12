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

#include "src/signal/array.hpp"
#include "integrator_base.hpp"

namespace pooya
{

template <typename T>
class TriggeredIntegratorT : public IntegratorBaseT<T>
{
protected:
    BoolSignalId _trigger;
    bool _triggered{false};

public:
    TriggeredIntegratorT(const std::string& given_name, T ic = T(0.0)) : IntegratorBaseT<T>(given_name, ic, 2, 1) {}

    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + IntegratorBaseT<T>::full_name());
        if (!IntegratorBaseT<T>::init(parent, ibus, obus)) {return false;}

        _trigger = IntegratorBaseT<T>::bool_input_at("trigger");

        return true;
    }

    void pre_step(double t) override
    {
        pooya_trace("block: " + IntegratorBaseT<T>::full_name());
        if (_triggered)
        {
            if constexpr (std::is_same_v<T, Array>)
                {IntegratorBaseT<T>::_value.setZero();}
            else
                {IntegratorBaseT<T>::_value = 0;}
            _triggered = false;
        }
        IntegratorBaseT<T>::pre_step(t);
    }

    uint _process(double t, bool go_deep=true) override
    {
        pooya_trace("block: " + IntegratorBaseT<T>::full_name());
        if (IntegratorBaseT<T>::_processed || !_trigger->is_assigned()) {return 0;}

        if (!_triggered) {_triggered = _trigger->get();}

        return IntegratorBaseT<T>::_process(t, go_deep);
    }
};

using TriggeredIntegrator = TriggeredIntegratorT<double>;
using TriggeredIntegratorA = TriggeredIntegratorT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_TRIGGERED_INTEGRATOR_HPP__
