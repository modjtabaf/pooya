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

#include "bus_block_builder.hpp"

namespace pooya {

bool BusBlockBuilder::init(Parent &parent, BusId ibus, BusId obus)
{
    pooya_trace("block: " + full_name());
    if (!SingleInputOutputT<BusSpec>::init(parent, ibus, obus)) {return false;}

    pooya_verify(_s_in->spec() == _s_out->spec(), "Bus specs don't match!");

    return true;
}

void BusBlockBuilder::post_init()
{
    pooya_trace("block: " + full_name());
    const auto &bus_spec = _s_in->spec();
    _blocks.reserve(bus_spec.total_size());
    traverse_bus("", bus_spec);
}

void BusBlockBuilder::traverse_bus(const std::string& full_label, const BusSpec& bus_spec)
{
    pooya_trace("block: " + full_name());

    if (std::find(_excluded_labels.begin(), _excluded_labels.end(), full_label) != _excluded_labels.end()) {return;}

    for (const auto &wi : bus_spec._wires)
    {
        if (wi._bus)
        {
            traverse_bus(full_label + wi.label() + ".", *wi._bus);
        }
        else
        {
            auto label = full_label + wi.label();
            if (std::find(_excluded_labels.begin(), _excluded_labels.end(), label) == _excluded_labels.end())
                {block_builder(label, wi, _s_in->at(label), _s_out->at(label));}
        }
    }
}

} // namespace pooya
