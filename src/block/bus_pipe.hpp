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

#ifndef __POOYA_BLOCK_BUS_PIPE_HPP__
#define __POOYA_BLOCK_BUS_PIPE_HPP__

#include "bus_block_builder.hpp"

namespace pooya
{

class BusPipe : public BusBlockBuilder
{
public:
    explicit BusPipe(const std::string& given_name, const std::initializer_list<std::string>& excluded_labels={})
            : BusBlockBuilder(given_name, excluded_labels) {}

protected:
    void block_builder(const std::string& /*full_label*/, const BusSpec::WireInfo &wi,
        SignalId sig_in, SignalId sig_out) override;
};

} // namespace pooya

#endif // __POOYA_BLOCK_BUS_PIPE_HPP__
