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

BusSpec::WireInfo::WireInfo(const std::string& coded_label)
{
    if (coded_label.find("i:") == 0)
    {
        _label             = coded_label.substr(2);
        _single_value_type = BusSpec::SingleValueType::Int;
    }
    else if (coded_label.find("b:") == 0)
    {
        _label             = coded_label.substr(2);
        _single_value_type = BusSpec::SingleValueType::Bool;
    }
    else if (coded_label.find("f:") == 0)
    {
        _label             = coded_label.substr(2);
        _single_value_type = BusSpec::SingleValueType::Scalar;
    }
    else
    {
        _label             = coded_label;
        _single_value_type = BusSpec::SingleValueType::Scalar;
    }
}

void BusImpl::_set(std::size_t index, SignalImplPtr sig)
{
    pooya_trace("index: " + std::to_string(index));
    pooya_verify(index < _signals.size(), "bus wire index out of range!");
    auto& ns = _signals[index];
    pooya_verify(ns.second == nullptr, ns.first + ": cannot re-assign a bus wire!");
#if defined(POOYA_DEBUG)
    auto& wi = static_cast<const BusSpec&>(_spec)._wires[index];
    if (wi._bus)
    {
        pooya_verify_bus_spec(sig, *wi._bus);
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Scalar)
    {
        pooya_verify_scalar_signal(sig);
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Int)
    {
        pooya_verify_int_signal(sig);
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Bool)
    {
        pooya_verify_bool_signal(sig);
    }
    else
    {
        pooya_verify_array_signal_size(sig, wi._array_size);
    }
#endif // defined(POOYA_DEBUG)
    ns.second = sig;
}

SignalImplPtr BusImpl::at(const std::string& label) const
{
    pooya_trace("label: " + label);
    auto pos   = label.find(".");
    auto index = static_cast<const BusSpec&>(_spec).index_of(pos == std::string::npos ? label : label.substr(0, pos));
    pooya_verify(index < static_cast<const BusSpec&>(_spec)._wires.size(), label + ": label not found in bus!");
    auto sig = at(index).second;
    if (pos == std::string::npos)
    {
        return sig;
    }

    pooya_verify_bus(sig);
    return sig->as_bus().at(label.substr(pos + 1));
}

SignalImplPtr BusImpl::operator[](const std::string& label) const
{
    pooya_trace("label: " + label);
    auto pos = label.find(".");
    if (pos == std::string::npos)
    {
        return operator[](static_cast<const BusSpec&>(_spec).index_of(label)).second;
    }

    auto sig = operator[](static_cast<const BusSpec&>(_spec).index_of(label.substr(0, pos))).second;
    pooya_verify_bus(sig);
    return sig->as_bus()[label.substr(pos + 1)];
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
    SignalImplPtr sig = _sid->at(index).second;
    pooya_verify_value_signal(sig);
    return std::static_pointer_cast<ValueSignalImpl>(sig->shared_from_this());
}

FloatSignalImplPtr Bus::float_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index).second;
    pooya_verify_float_signal(sig);
    return std::static_pointer_cast<FloatSignalImpl>(sig->shared_from_this());
}

ScalarSignalImplPtr Bus::scalar_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index).second;
    pooya_verify_scalar_signal(sig);
    return std::static_pointer_cast<ScalarSignalImpl>(sig->shared_from_this());
}

IntSignalImplPtr Bus::int_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index).second;
    pooya_verify_int_signal(sig);
    return std::static_pointer_cast<IntSignalImpl>(sig->shared_from_this());
}

BoolSignalImplPtr Bus::bool_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index).second;
    pooya_verify_bool_signal(sig);
    return std::static_pointer_cast<BoolSignalImpl>(sig->shared_from_this());
}

ArraySignalImplPtr Bus::array_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index).second;
    pooya_verify_array_signal(sig);
    return std::static_pointer_cast<ArraySignalImpl>(sig->shared_from_this());
}

BusImplPtr Bus::bus_at(std::size_t index) const
{
    pooya_trace("index: " + index);
    SignalImplPtr sig = _sid->at(index).second;
    pooya_verify_bus(sig);
    return std::static_pointer_cast<BusImpl>(sig->shared_from_this());
}

} // namespace pooya
