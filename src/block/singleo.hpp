/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_BLOCK_SO_HPP__
#define __POOYA_BLOCK_SO_HPP__

#include "leaf.hpp"

namespace pooya
{

template<typename T, class Base = Leaf>
class SingleOutputT : public Base
{
protected:
    typename Types<T>::Signal _s_out;

    SingleOutputT(uint16_t num_iports = Block::NoIOLimit, uint16_t num_oports = 1) : Base(num_iports, num_oports)
    {
        pooya_verify(num_oports == 1, "One and only one output expected!");
    }
    SingleOutputT(Submodel* parent, std::string_view name = "", uint16_t num_iports = Block::NoIOLimit,
                  uint16_t num_oports = 1)
        : Base(parent, name, num_iports, num_oports)
    {
        pooya_verify(num_oports == 1, "One and only one output expected!");
    }

public:
    bool connect(const Bus& ibus, const Bus& obus) override
    {
        if (!Base::connect(ibus, obus))
        {
            return false;
        }
        _s_out.reset(Types<T>::as_signal_id(Base::_obus.at(0)));
        return true;
    }
};

} // namespace pooya

#endif // __POOYA_BLOCK_SO_HPP__
