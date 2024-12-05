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

#include "bus_block_builder.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

class Value
{
protected:
    const bool _is_scalar{true};
    double _scalar{0.0};
    Array _array;

public:
    Value(double v) : _scalar(v) {}
    Value(const Array& v) : _is_scalar(false), _array(v) {}

    bool is_scalar() const { return _is_scalar; }

    double as_scalar() const
    {
        pooya_trace0;
        pooya_verify(_is_scalar, "attempting to retrieve an array as a scalar!");
        return _scalar;
    }

    const Array& as_array() const
    {
        pooya_trace0;
        pooya_verify(!_is_scalar, "attempting to retrieve a scalar as an array!");
        return _array;
    }
};

class BusMemory : public BusBlockBuilder
{
public:
    using LabelValueMap = std::map<std::string, Value>;
    using LabelValue    = LabelValueMap::value_type;

protected:
    LabelValueMap _init_values;

public:
    BusMemory(const std::initializer_list<LabelValue>& l                = {},
              const std::initializer_list<std::string>& excluded_labels = {})
        : BusBlockBuilder(excluded_labels), _init_values(l)
    {
    }
    BusMemory(const ValidName& name, const std::initializer_list<LabelValue>& l = {},
              const std::initializer_list<std::string>& excluded_labels = {})
        : BusBlockBuilder(name, excluded_labels), _init_values(l)
    {
    }

protected:
    void block_builder(const std::string& full_label, const SignalImplPtr& sig_in,
                       const SignalImplPtr& sig_out) override;
    bool init(Submodel* parent = nullptr, const Bus& ibus = Bus(), const Bus& obus = Bus()) override;
};

} // namespace pooya

#endif // __POOYA_BLOCK_BUS_MEMORY_HPP__
