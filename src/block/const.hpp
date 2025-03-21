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

#ifndef __POOYA_BLOCK_CONST_HPP__
#define __POOYA_BLOCK_CONST_HPP__

#include "singleo.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class ConstT : public SingleOutputT<T>
{
protected:
    T _value;

public:
    explicit ConstT(const T& value) : SingleOutputT<T>(0), _value(value) {}
    ConstT(Submodel* parent, std::string_view name, const T& value) : SingleOutputT<T>(parent, name, 0), _value(value)
    {
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        SingleOutputT<T>::_s_out->set(_value);
    }
};

using Const  = ConstT<double>;
using ConstA = ConstT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_CONST_HPP__
