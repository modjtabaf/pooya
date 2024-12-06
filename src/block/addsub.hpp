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

#ifndef __POOYA_BLOCK_ADDSUB_HPP__
#define __POOYA_BLOCK_ADDSUB_HPP__

#include "singleo.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class AddSubT : public SingleOutputT<T>
{
protected:
    std::string _operators;
    T _initial;
    T _ret;

public:
    AddSubT(const std::string& operators, const T& initial = 0.0)
        : SingleOutputT<T>(Block::NoIOLimit, 1), _operators(operators), _initial(initial)
    {
    }
    AddSubT(Submodel* parent, const std::string& operators, const T& initial = 0.0)
        : SingleOutputT<T>(parent, Block::NoIOLimit, 1), _operators(operators), _initial(initial)
    {
    }

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        if (!SingleOutputT<T>::connect(ibus, obus))
        {
            return false;
        }

        pooya_verify(ibus.size() >= 1, SingleOutputT<T>::full_name().str() + " requires 1 or more input signals.");
        pooya_verify(_operators.size() == ibus.size(),
                     SingleOutputT<T>::full_name().str() + ": mismatch between input signals and operators.");

        return true;
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name().str());
        _ret          = _initial;
        const char* p = _operators.c_str();
        for (const auto& sig_key : SingleOutputT<T>::_ibus)
        {
            const auto& v = Types<T>::as_signal_info(SingleOutputT<T>::_ibus[sig_key]);
            if (*p == '+')
            {
                _ret += v;
            }
            else if (*p == '-')
            {
                _ret -= v;
            }
            else
            {
                assert(false);
            }
            p++;
        }
        SingleOutputT<T>::_s_out->set(_ret);
    }
};

using AddSub  = AddSubT<double>;
using AddSubA = AddSubT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_ADDSUB_HPP__
