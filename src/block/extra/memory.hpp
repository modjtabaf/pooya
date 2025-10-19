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

#ifndef __POOYA_BLOCK_MEMORY_HPP__
#define __POOYA_BLOCK_MEMORY_HPP__

#include "src/block/singleio.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class MemoryT : public SingleInputOutputT<T>
{
public:
    using Base = SingleInputOutputT<T>;

    explicit MemoryT(typename Types<T>::SetValue ic, Submodel* parent = nullptr, std::string_view name = "")
        : Base(parent, name, 1), _value(ic)
    {
    }

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (!Base::connect(ibus, obus))
        {
            return false;
        }

        if (auto* ptr = Base::find_linked_signal(Base::_s_in.impl()))
        {
            ptr->second = ptr->second & (~Base::SignalLinkType::Required);
        }

        return true;
    }

    void post_step(double /*t*/) override
    {
        pooya_trace("block: " + Base::full_name().str());
        _value = Base::_s_in;
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + Base::full_name().str());
        Base::_s_out = _value;
    }

protected:
    T _value;
};

using Memory = MemoryT<double>;

#ifdef POOYA_INT_SIGNAL
using MemoryI = MemoryT<int>;
#endif // POOYA_INT_SIGNAL

#ifdef POOYA_BOOL_SIGNAL
using MemoryB = MemoryT<bool>;
#endif // POOYA_BOOL_SIGNAL

#ifdef POOYA_ARRAY_SIGNAL
using MemoryA = MemoryT<Array>;
#endif // POOYA_ARRAY_SIGNAL

} // namespace pooya

#endif // __POOYA_BLOCK_MEMORY_HPP__
