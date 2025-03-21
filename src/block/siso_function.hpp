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

#ifndef __POOYA_BLOCK_SISO_FUNCTION_HPP__
#define __POOYA_BLOCK_SISO_FUNCTION_HPP__

#include "singleio.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class SISOFunctionT : public SingleInputOutputT<T>
{
public:
    using ActFunction = std::function<typename Types<T>::SetValue(double, typename Types<T>::GetValue)>;

protected:
    ActFunction _act_func;

public:
    explicit SISOFunctionT(ActFunction act_func) : SingleInputOutputT<T>(1), _act_func(act_func) {}
    SISOFunctionT(Submodel* parent, std::string_view name, ActFunction act_func)
        : SingleInputOutputT<T>(parent, name, 1), _act_func(act_func)
    {
    }

    void activation_function(double t) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name().str());
        SingleInputOutputT<T>::_s_out->set(_act_func(t, SingleInputOutputT<T>::_s_in->get()));
    }
};

using SISOFunction  = SISOFunctionT<double>;
using SISOFunctionA = SISOFunctionT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_SISO_FUNCTION_HPP__
