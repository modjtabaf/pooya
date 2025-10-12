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

#ifndef __POOYA_BLOCK_LEAF_HPP__
#define __POOYA_BLOCK_LEAF_HPP__

#include "src/block/block.hpp"

namespace pooya
{

class Leaf : public Block
{
protected:
    explicit Leaf(Submodel* parent = nullptr, std::string_view name = "", uint16_t num_iports = NoIOLimit,
                  uint16_t num_oports = NoIOLimit)
        : Block(parent, name, num_iports, num_oports)
    {
    }

public:
    virtual ~Leaf() = default;

    uint process(double t, bool go_deep = true) override;

    virtual void activation_function(double /*t*/) {}
}; // class Leaf

} // namespace pooya

#endif // __POOYA_BLOCK_LEAF_HPP__
