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

#ifndef __POOYA_BLOCK_SI_HPP__
#define __POOYA_BLOCK_SI_HPP__

#include "block.hpp"

namespace pooya
{

template<typename T>
class SingleInputT : public Block
{
protected:
    typename Types<T>::SignalId _s_in{nullptr};

    SingleInputT(const std::string& given_name, uint16_t num_iports=1, uint16_t num_oports=NoIOLimit) :
        Block(given_name, num_iports, num_oports)
    {
        pooya_verify(num_iports == 1, "One and only one input expected!");
    }

public:
    bool init(Parent* parent, BusId ibus, BusId obus) override
    {
        if (!Block::init(parent, ibus, obus)) {return false;}
        _s_in = std::move(Types<T>::as_signal_id(_ibus->at(0).second));
        return true;
    }
};

}

#endif // __POOYA_BLOCK_SI_HPP__
