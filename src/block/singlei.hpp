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

#ifndef __POOYA_BLOCK_SI_HPP__
#define __POOYA_BLOCK_SI_HPP__

#include "leaf.hpp"

namespace pooya
{

template<typename T>
class SingleInputT : public Leaf
{
protected:
    typename Types<T>::Signal _s_in;

    explicit SingleInputT(Submodel* parent = nullptr, uint16_t num_oports = NoIOLimit) : Leaf(parent, 1, num_oports) {}
    SingleInputT(const ValidName& name, Submodel* parent = nullptr, uint16_t num_oports = NoIOLimit)
        : Leaf(name, parent, 1, num_oports)
    {
    }
    SingleInputT(Submodel* parent, uint16_t num_iports, uint16_t num_oports) : Leaf(parent, num_iports, num_oports)
    {
        pooya_verify(num_iports == 1, "One and only one input expected!");
    }
    SingleInputT(const ValidName& name, Submodel* parent, uint16_t num_iports, uint16_t num_oports)
        : Leaf(name, parent, num_iports, num_oports)
    {
        pooya_verify(num_iports == 1, "One and only one input expected!");
    }

public:
    bool init(const Bus& ibus, const Bus& obus) override
    {
        if (!Leaf::init(ibus, obus))
        {
            return false;
        }
        _s_in.reset(Types<T>::as_signal_id(_ibus.at(0)));
        return true;
    }
};

} // namespace pooya

#endif // __POOYA_BLOCK_SI_HPP__
