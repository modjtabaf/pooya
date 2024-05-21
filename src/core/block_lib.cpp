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

#include "block_lib.hpp"
#include "signal.hpp"

namespace pooya {

template <>
void SinT<double>::activation_function(double /*t*/)
{
    pooya_trace("block: " + full_name());
    _s_out->set(std::sin(_s_in->get()));
}

bool BusBlockBuilder::init(Parent &parent, BusId ibus, BusId obus)
{
    pooya_trace("block: " + full_name());
    if (!SingleInputOutputT<BusSpec>::init(parent, ibus, obus))
        return false;

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

    if (std::find(_excluded_labels.begin(), _excluded_labels.end(), full_label) != _excluded_labels.end())
        return;

    for (const auto &wi : bus_spec._wires)
    {
        if (wi._bus)
          traverse_bus(full_label + wi.label() + ".", *wi._bus);
        else
        {
            auto label = full_label + wi.label();
            if (std::find(_excluded_labels.begin(), _excluded_labels.end(), label) == _excluded_labels.end())
                block_builder(label, wi, _s_in->at(label), _s_out->at(label));
        }
    }
}

void BusMemory::post_init()
{
    pooya_trace("block: " + full_name());
    BusBlockBuilder::post_init();
    pooya_verify(_init_values.empty(), full_name() + ": Some initial values of bus memory block were not used!");
}

void BusMemory::block_builder(const std::string &full_label, const BusSpec::WireInfo &wi,
  SignalId sig_in, SignalId sig_out)
{
    pooya_trace("block: " + full_name());
    auto it = _init_values.find(full_label);

    std::shared_ptr<Block> block;
    if (wi.single_value_type() == BusSpec::SingleValueType::Scalar)
    {
        block = (it == _init_values.end())
            ? std::make_shared<Memory>("memory")
            : std::make_shared<Memory>("memory", it->second.as_scalar());
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Int)
    {
        block = (it == _init_values.end())
            ? std::make_shared<MemoryI>("memory")
            : std::make_shared<MemoryI>("memory", std::round(it->second.as_scalar()));
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Bool)
    {
        block = (it == _init_values.end())
            ? std::make_shared<MemoryB>("memory")
            : std::make_shared<MemoryB>("memory", it->second.as_scalar() != 0);
    }
    else if (wi._array_size > 0)
    {
        block = (it == _init_values.end())
            ? std::make_shared<MemoryA>("memory", Array::Zero(wi._array_size))
            : std::make_shared<MemoryA>("memory", it->second.as_array());
    }
    else
    {
        pooya_verify(false, "cannot create a memory block for a non-value signal.");
    }

    if (it != _init_values.end())
    {
        _init_values.erase(it);
    }

#if defined(POOYA_USE_SMART_PTRS)
    auto& parent = _parent->get();
#else // defined(POOYA_USE_SMART_PTRS)
    auto& parent = *_parent;
#endif // defined(POOYA_USE_SMART_PTRS)
    parent.add_block(*block, sig_in, sig_out);
    _blocks.push_back(std::move(block));
}

void BusPipe::block_builder(const std::string & /*full_label*/, const BusSpec::WireInfo &wi,
    SignalId sig_in, SignalId sig_out)
{
    pooya_trace("block: " + full_name());
    std::shared_ptr<Block> block;
    if (wi.single_value_type() == BusSpec::SingleValueType::Scalar)
    {
        block = std::make_shared<Pipe>("pipe");
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Int)
    {
        block = std::make_shared<PipeI>("pipe");
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Bool)
    {
        block = std::make_shared<PipeB>("pipe");
    }
    else if (wi._array_size > 0)
    {
        block = std::make_shared<PipeA>("pipe");
    }
    else
    {
        pooya_verify(false, "cannot create a pipe block for a non-value signal.");
    }

#if defined(POOYA_USE_SMART_PTRS)
    auto& parent = _parent->get();
#else // defined(POOYA_USE_SMART_PTRS)
    auto& parent = *_parent;
#endif // defined(POOYA_USE_SMART_PTRS)
    parent.add_block(*block, sig_in, sig_out);
    _blocks.push_back(std::move(block));
}

} // namespace pooya
