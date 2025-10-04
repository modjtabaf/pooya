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

#ifndef __POOYA_BLOCK_ADD_HPP__
#define __POOYA_BLOCK_ADD_HPP__

#include "src/block/singleo.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class AddT : public SingleOutputT<T>
{
public:
    using Base = SingleOutputT<T>;

    explicit AddT(typename Types<T>::SetValue initial = 0.0, Submodel* parent = nullptr, std::string_view name = "")
        : Base(parent, name, Block::NoIOLimit, 1), _initial(initial)
    {
    }

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (!Base::connect(ibus, obus))
        {
            return false;
        }

        pooya_debug_verify(ibus->size() >= 1, Base::full_name().str() + " requires 1 or more input signals.");
        for (const auto& sig : ibus)
            pooya_debug_verify(dynamic_cast<typename Types<T>::SignalImpl*>(&sig.second.impl()),
                               Base::full_name().str() + ": signal type mismatch!");

        return true;
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + Base::full_name().str());
        _ret = _initial;
        for (const auto& sig : Base::_ibus)
        {
            _ret += static_cast<const typename Types<T>::SignalImpl*>(&sig.second.impl())->get_value();
        }
        Base::_s_out = _ret;
    }

protected:
    T _initial;
    T _ret;
};

using Add = AddT<double>;

#ifdef POOYA_ARRAY_SIGNAL
using AddA = AddT<Array>;
#endif // POOYA_ARRAY_SIGNAL

} // namespace pooya

#endif // __POOYA_BLOCK_ADD_HPP__
