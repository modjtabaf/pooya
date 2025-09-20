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

namespace pooya
{

bool BusBlockBuilder::connect(const Bus& ibus, const Bus& obus)
{
    pooya_trace("block: " + full_name().str());
    if (!Leaf::connect(ibus, obus))
    {
        return false;
    }

    _blocks.reserve(ibus->size());
    visit_bus("", ibus);

    return true;
}

void BusBlockBuilder::visit_bus(const std::string& full_label, const Bus& bus)
{
    pooya_trace("block: " + full_name().str());

    if (std::find(_excluded_labels.begin(), _excluded_labels.end(), full_label) != _excluded_labels.end())
    {
        return;
    }

    for (const auto& [label, sig] : *bus)
    {
        if (!sig)
            continue;
        else if (auto* pa = dynamic_cast<pooya::BusImpl*>(sig.get()); pa)
        {
            visit_bus(full_label + label + ".", pa);
        }
        else
        {
            auto new_label = full_label + label;
            if (std::find(_excluded_labels.begin(), _excluded_labels.end(), new_label) == _excluded_labels.end())
            {
                block_builder(new_label, _ibus->at(new_label), _obus->at(new_label));
            }
        }
    }
}

void BusBlockBuilder::_mark_unprocessed()
{
    _processed = true;
}

} // namespace pooya
