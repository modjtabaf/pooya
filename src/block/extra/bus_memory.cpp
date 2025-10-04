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

#include "bus_memory.hpp"
#include "memory.hpp"
#include "src/block/submodel.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/scalar_signal.hpp"

namespace pooya
{

bool BusMemory::connect(const Bus& ibus, const Bus& obus)
{
    pooya_trace("block: " + full_name().str());
    if (!BusBlockBuilder::connect(ibus, obus))
    {
        return false;
    }

#ifdef POOYA_DEBUG
    if (!_init_values.empty())
    {
        std::stringstream ss;
        ss << full_name().str() << ": The following initial values of bus memory block were not used:\n";
        for (const auto& iv : _init_values)
        {
            ss << " - " << iv.first << "\n";
        }
        pooya_debug_verify(false, ss.str());
    }
#endif // POOYA_DEBUG

    return true;
}

void BusMemory::block_builder(const std::string& full_label, const Signal& sig_in, const Signal& sig_out)
{
    pooya_trace("block: " + full_name().str());
    auto it = _init_values.find(full_label);

    std::shared_ptr<Block> block;
    if (dynamic_cast<ScalarSignalImpl*>(&sig_in.impl()))
    {
        block = std::make_shared<Memory>((it == _init_values.end()) ? 0.0 : it->second.as_scalar(), _parent);
    }
#ifdef POOYA_INT_SIGNAL
    else if (dynamic_cast<IntSignalImpl*>(&sig_in.impl()))
    {
        block = std::make_shared<MemoryI>((it == _init_values.end()) ? 0 : std::round(it->second.as_scalar()), _parent);
    }
#endif // POOYA_INT_SIGNAL
#ifdef POOYA_BOOL_SIGNAL
    else if (dynamic_cast<BoolSignalImpl*>(&sig_in.impl()))
    {
        block =
            std::make_shared<MemoryB>(Bool((it == _init_values.end()) ? false : it->second.as_scalar() != 0), _parent);
    }
#endif // POOYA_BOOL_SIGNAL
#ifdef POOYA_ARRAY_SIGNAL
    else if (auto* pa = dynamic_cast<ArraySignalImpl*>(&sig_in.impl()); pa)
    {
        block = std::make_shared<MemoryA>((it == _init_values.end()) ? Array::Zero(pa->size()) : it->second.as_array(),
                                          _parent);
    }
#endif // POOYA_ARRAY_SIGNAL
    else
    {
        pooya_verify(false, "cannot create a memory block for a non-value signal.");
    }

    if (it != _init_values.end())
    {
        _init_values.erase(it);
    }

    block->rename(full_label);
    block->connect({sig_in}, {sig_out});
    _blocks.emplace_back(std::move(block));
}

} // namespace pooya
