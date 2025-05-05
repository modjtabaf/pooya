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

#include "singleo.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class MulDivT : public SingleOutputT<T>
{
public:
    using Base = SingleOutputT<T>;

protected:
    std::string _operators;
    T _initial;
    T _ret;

public:
    MulDivT(std::string_view operators, const T& initial = 1)
        : Base(Block::NoIOLimit, 1), _operators(operators), _initial(initial)
    {
    }
    MulDivT(Submodel* parent, std::string_view name, std::string_view operators, const T& initial = 1)
        : Base(parent, name, Block::NoIOLimit, 1), _operators(operators), _initial(initial)
    {
    }

#if __cplusplus >= 202002L // C++20
    struct Params
    {
        Submodel* parent{nullptr};
        std::string_view name{""};
        T initial{1};
    };
    static_assert(std::is_aggregate_v<Params>);

    explicit MulDivT(std::string_view operators, const Params& params = {})
        : Base({.parent = params.parent, .name = params.name}), _operators(operators), _initial(params.initial)
    {
    }
#endif // __cplusplus >= 202002L // C++20

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + Base::full_name().str());
        if (!Base::connect(ibus, obus))
        {
            return false;
        }

        pooya_verify(ibus.size() >= 1, "MulDivT requires 1 or more input signals.");
        pooya_verify(_operators.size() == ibus.size(),
                     Base::full_name().str() + ": mismatch between input signals and operators.");

        return true;
    }

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + Base::full_name().str());
        _ret          = _initial;
        const char* p = _operators.c_str();
        for (const auto& sig_key : Base::_ibus)
        {
            const auto& v = Types<T>::as_signal_info(Base::_ibus[sig_key]).get();
            if (*p == '*')
            {
                _ret *= v;
            }
            else if (*p == '/')
            {
                _ret /= v;
            }
            else
            {
                assert(false);
            }
            p++;
        }
        Base::_s_out->set(_ret);
    }
};

using MulDiv  = MulDivT<double>;
using MulDivA = MulDivT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_MULDIV_HPP__
