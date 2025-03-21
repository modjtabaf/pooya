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
#include "src/signal/array_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/scalar_signal.hpp"
#include "submodel.hpp"

namespace pooya
{

bool BusMemory::connect(const Bus& ibus, const Bus& obus)
{
    pooya_trace("block: " + full_name().str());
    if (!BusBlockBuilder::connect(ibus, obus))
    {
        return false;
    }

    pooya_verify(_init_values.empty(), full_name().str() + ": Some initial values of bus memory block were not used!");

    return true;
}

void BusMemory::block_builder(const std::string& full_label, const SignalImplPtr& sig_in, const SignalImplPtr& sig_out)
{
    pooya_trace("block: " + full_name().str());
    auto it = _init_values.find(full_label);

    std::shared_ptr<Block> block;
    if (sig_in->is_scalar())
    {
        block = (it == _init_values.end()) ? std::make_shared<Memory>(_parent)
                                           : std::make_shared<Memory>(_parent, "", it->second.as_scalar());
    }
    else if (sig_in->is_int())
    {
        block = (it == _init_values.end()) ? std::make_shared<MemoryI>(_parent)
                                           : std::make_shared<MemoryI>(_parent, "", std::round(it->second.as_scalar()));
    }
    else if (sig_in->is_bool())
    {
        block = (it == _init_values.end()) ? std::make_shared<MemoryB>(_parent)
                                           : std::make_shared<MemoryB>(_parent, "", Bool(it->second.as_scalar() != 0));
    }
    else if (sig_in->is_array())
    {
        block = (it == _init_values.end())
                    ? std::make_shared<MemoryA>(_parent, "", Array::Zero(sig_in->as_array().size()))
                    : std::make_shared<MemoryA>(_parent, "", it->second.as_array());
    }
    else
    {
        pooya_verify(false, "cannot create a memory block for a non-value signal.");
    }

    if (it != _init_values.end())
    {
        _init_values.erase(it);
    }

    block->connect(sig_in, sig_out);
    _blocks.emplace_back(std::move(block));
}

} // namespace pooya
