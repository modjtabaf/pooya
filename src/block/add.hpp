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

#include "singleo.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class AddT : public SingleOutputT<T>
{
protected:
    T _initial;
    T _ret;

public:
    AddT(const T& initial = 0.0) : SingleOutputT<T>(Block::NoIOLimit, 1), _initial(initial) {}
    AddT(Submodel* parent, const T& initial = 0.0) : SingleOutputT<T>(parent, Block::NoIOLimit, 1), _initial(initial) {}

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (!SingleOutputT<T>::connect(ibus, obus))
        {
            return false;
        }

        pooya_verify(ibus.size() >= 1, SingleOutputT<T>::full_name().str() + " requires 1 or more input signals.");

        return true;
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        _ret = _initial;
        for (const auto& [label, sig] : SingleOutputT<T>::_ibus)
        {
            _ret += Types<T>::as_signal_info(sig).get();
        }
        SingleOutputT<T>::_s_out->set(_ret);
    }
};

using Add  = AddT<double>;
using AddA = AddT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_ADD_HPP__
