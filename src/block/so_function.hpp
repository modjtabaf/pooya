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

#ifndef __POOYA_BLOCK_SO_FUNCTION_HPP__
#define __POOYA_BLOCK_SO_FUNCTION_HPP__

#include "singleo.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class SOFunctionT : public SingleOutputT<T>
{
public:
    using ActFunction = std::function<typename Types<T>::SetValue(double, const pooya::Bus& ibus)>;

protected:
    ActFunction _act_func;

public:
    SOFunctionT(Submodel* parent, ActFunction act_func, uint16_t num_iports = Block::NoIOLimit)
        : SingleOutputT<T>(parent, num_iports), _act_func(act_func)
    {
    }
    SOFunctionT(Submodel* parent, ActFunction act_func, uint16_t num_iports = Block::NoIOLimit)
        : SingleOutputT<T>(parent, num_iports), _act_func(act_func)
    {
    }

    void activation_function(double t) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        SingleOutputT<T>::_s_out->set(_act_func(t, SingleOutputT<T>::_ibus));
    }
};

using SOFunction  = SOFunctionT<double>;
using SOFunctionA = SOFunctionT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_SO_FUNCTION_HPP__
