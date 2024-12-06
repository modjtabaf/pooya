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

    for (const auto& sig_key : bus)
    {
        const auto& sig = bus[sig_key];
        if (sig->is_bus())
        {
            visit_bus(full_label + sig_key + ".", Bus(sig->shared_from_this()));
        }
        else
        {
            auto label = full_label + sig_key;
            if (std::find(_excluded_labels.begin(), _excluded_labels.end(), label) == _excluded_labels.end())
            {
                block_builder(label, _ibus.at(label), _obus.at(label));
            }
        }
    }
}

void BusBlockBuilder::_mark_unprocessed()
{
    _processed = true;
}

} // namespace pooya
