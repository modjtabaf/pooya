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

#ifndef __POOYA_BLOCK_BUS_MEMORY_HPP__
#define __POOYA_BLOCK_BUS_MEMORY_HPP__

#include <map>
#include <variant>

#include "src/signal/array.hpp"
#include "bus_block_builder.hpp"

namespace pooya
{

class BusMemory : public BusBlockBuilder
{
public:
    using LabelValueMap = std::map<std::string, std::variant<double, Array>>;
    using LabelValue = LabelValueMap::value_type;

protected:
    LabelValueMap _init_values;

public:
    BusMemory(const std::string& given_name, const std::initializer_list<LabelValue> &l = {}, const std::initializer_list<std::string>& excluded_labels={})
            : BusBlockBuilder(given_name, excluded_labels), _init_values(l) {}

protected:
    void block_builder(const std::string& full_label, const BusSpec::WireInfo &wi, SignalId sig_in, SignalId sig_out) override;
    void post_init() override;
};

} // namespace pooya

#endif // __POOYA_BLOCK_BUS_MEMORY_HPP__
