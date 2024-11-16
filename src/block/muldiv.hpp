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

#ifndef __POOYA_BLOCK_MULDIV_HPP__
#define __POOYA_BLOCK_MULDIV_HPP__

#include "src/signal/array.hpp"
#include "singleo.hpp"

namespace pooya
{

template <typename T>
class MulDivT : public SingleOutputT<T>
{
protected:
    std::string _operators;
    T _initial;
    T _ret;

    bool init(Submodel* parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (!SingleOutputT<T>::init(parent, ibus, obus)) {return false;}

        pooya_verify(ibus->size() >= 1, "MulDivT requires 1 or more input signals.");
        pooya_verify(_operators.size() == ibus->size(),
                     SingleOutputT<T>::full_name() + ": mismatch between input signals and operators.");

        return true;
    }

public:
    MulDivT(const std::string& given_name, const char *operators, const T &initial = 1.0)
            : SingleOutputT<T>(given_name, Block::NoIOLimit, 1), _operators(operators), _initial(initial) {}

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        _ret = _initial;
        const char *p = _operators.c_str();
        for (const auto &ls: *SingleOutputT<T>::_ibus)
        {
            const auto &v = Types<T>::as_signal_info(ls.second).get();
            if (*p == '*') {_ret *= v;}
            else if (*p == '/') {_ret /= v;}
            else {assert(false);}
            p++;
        }
        SingleOutputT<T>::_s_out->set(_ret);
    }
};

using MulDiv = MulDivT<double>;
using MulDivA = MulDivT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_MULDIV_HPP__
