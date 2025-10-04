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

#include "src/block/singleo.hpp"

namespace pooya
{

template<typename T>
class IntegratorBaseT : public SingleOutputT<T>
{
public:
    using Base = SingleOutputT<T>;

    explicit IntegratorBaseT(typename Types<T>::SetValue ic = 0.0, Submodel* parent = nullptr,
                             std::string_view name = "", uint16_t num_iports = Block::NoIOLimit,
                             uint16_t num_oports = Block::NoIOLimit)
        : Base(parent, name, num_iports, num_oports), _value(ic)
    {
    }

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (!Base::connect(ibus, obus))
        {
            return false;
        }

        Base::_s_out->set_deriv_signal(Base::input(0));

        return true;
    }

    void pre_step(double /*t*/) override
    {
        pooya_trace("block: " + Base::full_name().str());
        pooya_debug_verify0(!Base::_s_out->is_assigned());
        Base::_s_out = _value;
    }

    void post_step(double /*t*/) override
    {
        pooya_trace("block: " + Base::full_name().str());
        pooya_debug_verify0(Base::_s_out->is_assigned());
        _value = Base::_s_out;
    }

    uint _process(double /*t*/, bool /*go_deep*/ = true) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (Base::_processed)
        {
            return 0;
        }

        Base::_processed = Base::_s_out->is_assigned();
        return Base::_processed ? 1 : 0; // is it safe to simply return _processed?
    }

protected:
    T _value;
};

} // namespace pooya

#endif // __POOYA_BLOCK_INTEGRATOR_BASE_HPP__
