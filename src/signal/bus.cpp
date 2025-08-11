/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "bus.hpp"
#include "array_signal.hpp"
#include "bool_signal.hpp"
#include "int_signal.hpp"
#include "scalar_signal.hpp"
#include "value_signal.hpp"
#include <memory>

namespace pooya
{

const SignalImplPtr& BusImpl::at(const std::string& label) const
{
    pooya_trace("label: " + label);

    auto pos                  = label.find(".");
    const auto& label_to_find = pos == std::string::npos ? label : label.substr(0, pos);
    const auto it             = std::find_if(_signals.begin(), _signals.end(),
                                             [label_to_find](const auto& p) -> bool { return p.first == label_to_find; });
    if (it == _signals.end())
    {
        throw std::out_of_range("Label not found in the bus: " + label);
    }
    if (pos == std::string::npos)
    {
        return it->second;
    }

    pooya_verify_bus(it->second);
    return it->second->as_bus().at(label.substr(pos + 1));
}

ValueSignalImplPtr Bus::value_at(const std::string& label) const
{
    pooya_trace("label: " + label);
    SignalImplPtr sig = _sid->at(label);
    pooya_verify_value_signal(sig);
    return std::static_pointer_cast<ValueSignalImpl>(sig->shared_from_this());
}

FloatSignalImplPtr Bus::float_at(const std::string& label) const
{
    pooya_trace("label: " + label);
    SignalImplPtr sig = _sid->at(label);
    pooya_verify_float_signal(sig);
    return std::static_pointer_cast<FloatSignalImpl>(sig->shared_from_this());
}

ScalarSignalImplPtr Bus::scalar_at(const std::string& label) const
{
    pooya_trace("label: " + label);
    SignalImplPtr sig = _sid->at(label);
    pooya_verify_scalar_signal(sig);
    return std::static_pointer_cast<ScalarSignalImpl>(sig->shared_from_this());
}

IntSignalImplPtr Bus::int_at(const std::string& label) const
{
    pooya_trace("label: " + label);
    SignalImplPtr sig = _sid->at(label);
    pooya_verify_int_signal(sig);
    return std::static_pointer_cast<IntSignalImpl>(sig->shared_from_this());
}

BoolSignalImplPtr Bus::bool_at(const std::string& label) const
{
    pooya_trace("label: " + label);
    SignalImplPtr sig = _sid->at(label);
    pooya_verify_bool_signal(sig);
    return std::static_pointer_cast<BoolSignalImpl>(sig->shared_from_this());
}

ArraySignalImplPtr Bus::array_at(const std::string& label) const
{
    pooya_trace("label: " + label);
    SignalImplPtr sig = _sid->at(label);
    pooya_verify_array_signal(sig);
    return std::static_pointer_cast<ArraySignalImpl>(sig->shared_from_this());
}

BusImplPtr Bus::bus_at(const std::string& label) const
{
    pooya_trace("label: " + label);
    SignalImplPtr sig = _sid->at(label);
    pooya_verify_bus(sig);
    return std::static_pointer_cast<BusImpl>(sig->shared_from_this());
}

ValueSignalImplPtr Bus::value_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index);
    pooya_verify_value_signal(sig);
    return std::static_pointer_cast<ValueSignalImpl>(sig->shared_from_this());
}

FloatSignalImplPtr Bus::float_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index);
    pooya_verify_float_signal(sig);
    return std::static_pointer_cast<FloatSignalImpl>(sig->shared_from_this());
}

ScalarSignalImplPtr Bus::scalar_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index);
    pooya_verify_scalar_signal(sig);
    return std::static_pointer_cast<ScalarSignalImpl>(sig->shared_from_this());
}

IntSignalImplPtr Bus::int_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index);
    pooya_verify_int_signal(sig);
    return std::static_pointer_cast<IntSignalImpl>(sig->shared_from_this());
}

BoolSignalImplPtr Bus::bool_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index);
    pooya_verify_bool_signal(sig);
    return std::static_pointer_cast<BoolSignalImpl>(sig->shared_from_this());
}

ArraySignalImplPtr Bus::array_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index);
    pooya_verify_array_signal(sig);
    return std::static_pointer_cast<ArraySignalImpl>(sig->shared_from_this());
}

BusImplPtr Bus::bus_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index);
    pooya_verify_bus(sig);
    return std::static_pointer_cast<BusImpl>(sig->shared_from_this());
}

} // namespace pooya
