/*
Copyright 2023 Mojtaba (Moji) Fathi

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

#include <algorithm>
#include <iostream>

#include "block_lib.hpp"

namespace pooya {

template <>
void SinT<double>::activation_function(double /*t*/, Values &values)
{
    pooya_trace("block: " + full_name());
    values.set<double>(_oports[0], std::sin(values.get<double>(_iports[0])));
}

bool BusBlockBuilder::init(Parent &parent, const LabelSignals& iports, const LabelSignals& oports)
{
    pooya_trace("block: " + full_name());
    if (!Block::init(parent, iports, oports))
        return false;

    _iports.bind(_x);
    _oports.bind(_y);

    verify(_x->spec() == _y->spec(), "Bus specs don't match!");

    return true;
}

void BusBlockBuilder::post_init()
{
    pooya_trace("block: " + full_name());
    const auto &bus_spec = _x->spec();
    _blocks.reserve(bus_spec.total_size());
    traverse_bus("", bus_spec);
}

void BusBlockBuilder::traverse_bus(const std::string &full_label, const BusSpec &bus_spec)
{
    pooya_trace("block: " + full_name());
    for (const auto &wi : bus_spec._wires)
    {
        if (wi._bus)
          traverse_bus(full_label + wi._label + ".", *wi._bus);
        else
        {
          auto label = full_label + wi._label;
          block_builder(label, wi, _x->at(label), _y->at(label));
        }
    }
}

void BusMemory::post_init()
{
    pooya_trace("block: " + full_name());
    BusBlockBuilder::post_init();
    verify(_init_values.empty(), full_name() + ": Some initial values of bus memory block were not used!");
}

void BusMemory::block_builder(const std::string &full_label, const BusSpec::WireInfo &wi,
  Signal sig_in, Signal sig_out)
{
    pooya_trace("block: " + full_name());
    auto it = _init_values.find(full_label);

    std::unique_ptr<Block> block;
    if (wi._scalar)
    {
        block = (it == _init_values.end())
            ? std::make_unique<Memory>("memory")
            : std::make_unique<Memory>("memory", it->second.as_scalar());
    }
    else if (wi._array_size > 0)
    {
        block = (it == _init_values.end())
            ? std::make_unique<MemoryA>("memory")
            : std::make_unique<MemoryA>("memory", it->second.as_array());
    }
    else
    {
        verify(false, "cannot create a memory block for a non-value signal.");
    }

    if (it != _init_values.end())
    {
        _init_values.erase(it);
    }

    _parent->add_block(*block, sig_in, sig_out);
    _blocks.push_back(std::move(block));
}

void BusPipe::block_builder(const std::string & /*full_label*/, const BusSpec::WireInfo &wi,
    Signal sig_in, Signal sig_out)
{
    pooya_trace("block: " + full_name());
    std::unique_ptr<Block> block;
    if (wi._scalar) {
        block = std::make_unique<Pipe>("pipe");
    }
    else if (wi._array_size > 0)
    {
        block = std::make_unique<PipeA>("pipe");
    }
    else
    {
        verify(false, "cannot create a pipe block for a non-value signal.");
    }

    _parent->add_block(*block, sig_in, sig_out);
    _blocks.push_back(std::move(block));
}

} // namespace pooya
