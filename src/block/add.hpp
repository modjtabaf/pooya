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

#include "addsub.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class AddT : public AddSubT<T>
{
public:
    using Base = AddSubT<T>;

    explicit AddT(const T& initial = 0) : Base("", initial) {}
    AddT(Submodel* parent, std::string_view name = "", const T& initial = 0) : Base(parent, name, "", initial) {}

#if __cplusplus >= 202002L // C++20
    explicit AddT(const Base::Params& params) : Base("", params) {}
#endif // __cplusplus >= 202002L // C++20

    bool connect(const Bus& ibus, const Bus& obus) override
    {
        pooya_trace("block: " + Base::full_name().str());
        Base::_operators = std::string(ibus.size(), '+');
        return Base::connect(ibus, obus);
    }
};

using Add  = AddT<double>;
using AddA = AddT<Array>;

} // namespace pooya

#endif // __POOYA_BLOCK_ADD_HPP__
