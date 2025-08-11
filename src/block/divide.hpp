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

#ifndef __POOYA_BLOCK_DIVIDE_HPP__
#define __POOYA_BLOCK_DIVIDE_HPP__

#include "singleo.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class DivideT : public SingleOutputT<T>
{
protected:
    // input signals
    typename Types<T>::Signal _s_x1; // input 1
    typename Types<T>::Signal _s_x2; // input 2

public:
    DivideT() : SingleOutputT<T>(2, 1) {}
    DivideT(Submodel* parent) : SingleOutputT<T>(parent, 2, 1) {}

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (!SingleOutputT<T>::connect(ibus, obus))
        {
            return false;
        }

        // input signals
        _s_x1.reset(Types<T>::as_signal_id(SingleOutputT<T>::_ibus.at(0)));
        _s_x2.reset(Types<T>::as_signal_id(SingleOutputT<T>::_ibus.at(1)));

        return true;
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        SingleOutputT<T>::_s_out->set(_s_x1 / _s_x2);
    }
};

using Divide  = DivideT<double>;
using DivideA = DivideT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_DIVIDE_HPP__
