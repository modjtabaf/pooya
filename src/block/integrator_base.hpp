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

#ifndef __POOYA_BLOCK_INTEGRATOR_BASE_HPP__
#define __POOYA_BLOCK_INTEGRATOR_BASE_HPP__

#include "singleo.hpp"

namespace pooya
{

template <typename T>
class IntegratorBaseT : public SingleOutputT<T>
{
protected:
    T _value;

public:
    IntegratorBaseT(T ic = T(0.0), uint16_t num_iports=Block::NoIOLimit, uint16_t num_oports=Block::NoIOLimit)
        : SingleOutputT<T>(num_iports, num_oports), _value(ic) {}
    IntegratorBaseT(const ValidName& name, T ic = T(0.0), uint16_t num_iports=Block::NoIOLimit, uint16_t num_oports=Block::NoIOLimit)
        : SingleOutputT<T>(name, num_iports, num_oports), _value(ic) {}

    bool init(Submodel* parent, const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (!SingleOutputT<T>::init(parent, ibus, obus)) {return false;}

        SingleOutputT<T>::_s_out->set_deriv_signal(Types<T>::as_signal_id(SingleOutputT<T>::_ibus.at(0).second));

        return true;
    }

    void pre_step(double /*t*/) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        assert(!SingleOutputT<T>::_s_out->is_assigned());
        SingleOutputT<T>::_s_out->set(_value);
    }

    void post_step(double /*t*/) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        assert(SingleOutputT<T>::_s_out->is_assigned());
        _value = SingleOutputT<T>::_s_out->get();
    }

    uint _process(double /*t*/, bool /*go_deep*/ = true) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (SingleOutputT<T>::_processed) {return 0;}

        SingleOutputT<T>::_processed = SingleOutputT<T>::_s_out->is_assigned();
        return SingleOutputT<T>::_processed ? 1 : 0; // is it safe to simply return _processed?
    }
};

} // namespace pooya

#endif // __POOYA_BLOCK_INTEGRATOR_BASE_HPP__
