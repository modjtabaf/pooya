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

#include "src/signal/array.hpp"
#include "singleio.hpp"

namespace pooya
{

template <typename T>
class MemoryT : public SingleInputOutputT<T>
{
protected:
    T _value;

public:
    MemoryT(const std::string& given_name, const T &ic = T(0))
            : SingleInputOutputT<T>(given_name, 1, 1), _value(ic) {}

    void post_step(double /*t*/) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        _value = SingleInputOutputT<T>::_s_in->get();
    }

    // # Memory can be implemented either by defining the following activation
    // function #   (which is more straightforward) or through overloading the
    // _process method #   which is more efficient since it deosn't rely on the
    // input signal being known. #   Both approaches are supposed to lead to the
    // exact same results.

    // # def activation_function(self, t, x):
    // #     return [self._value]

    uint _process(double /*t*/, bool /*go_deep*/) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        if (SingleInputOutputT<T>::_processed) {return 0;}

        pooya_trace_update0;
        SingleInputOutputT<T>::_s_out->set(_value);
        SingleInputOutputT<T>::_processed = true;
        return 1;
    }
};

using Memory = MemoryT<double>;
using MemoryI = MemoryT<int>;
using MemoryB = MemoryT<bool>;
using MemoryA = MemoryT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_MEMORY_HPP__
