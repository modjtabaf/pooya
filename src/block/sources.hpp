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

#ifndef __POOYA_BLOCK_SOURCES_HPP__
#define __POOYA_BLOCK_SOURCES_HPP__

#include "leaf.hpp"

namespace pooya
{

class Sources : public Leaf
{
public:
    using SourcesFunction = std::function<void(const Bus&, double)>;

protected:
    SourcesFunction _src_func;

public:
    Sources(SourcesFunction src_func, uint16_t num_oports = NoIOLimit) : Leaf(0, num_oports), _src_func(src_func) {}
    Sources(const ValidName& name, SourcesFunction src_func, uint16_t num_oports = NoIOLimit)
        : Leaf(name, 0, num_oports), _src_func(src_func)
    {
    }

    void activation_function(double t) override
    {
        pooya_trace("block: " + full_name().str());
        _src_func(_obus, t);
    }
};

} // namespace pooya

#endif // __POOYA_BLOCK_SOURCES_HPP__
