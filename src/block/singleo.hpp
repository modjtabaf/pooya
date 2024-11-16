/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_BLOCK_SO_HPP__
#define __POOYA_BLOCK_SO_HPP__

#include "block.hpp"

namespace pooya
{

template<typename T, class Base=Block>
class SingleOutputT : public Base
{
protected:
    typename Types<T>::SignalId _s_out{nullptr};

    SingleOutputT(const std::string& given_name, uint16_t num_iports=Block::NoIOLimit, uint16_t num_oports=1) :
        Base(given_name, num_iports, num_oports)
    {
        pooya_verify(num_oports == 1, "One and only one output expected!");
    }

public:
    bool init(Submodel* parent, BusId ibus, BusId obus) override
    {
        if (!Base::init(parent, ibus, obus)) {return false;}
        _s_out = std::move(Types<T>::as_signal_id(Base::_obus->at(0).second));
        return true;
    }
};

}

#endif // __POOYA_BLOCK_SO_HPP__
