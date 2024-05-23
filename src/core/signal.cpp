/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <cstddef>
#include <string>

#include "signal.hpp"
#include "util.hpp"

namespace pooya
{

void LabelSignals::_init(LabelSignalIdList::const_iterator begin_, LabelSignalIdList::const_iterator end_)
{
    pooya_trace0;
    for (auto it=begin_; it != end_; it++)
        push_back(*it);
}

LabelSignals::LabelSignals(const std::initializer_list<SignalId>& il)
{
    pooya_trace0;
    LabelSignalIdList lsl;
    std::size_t index=0;
    for (const auto& sig: il)
        lsl.push_back({_make_auto_label(index++), sig});
    _init(lsl.begin(), lsl.end());
}

LabelSignals::LabelSignals(const std::initializer_list<LabelSignalId>& il)
{
    pooya_trace0;
    LabelSignalIdList lsl(il);
    _init(lsl.begin(), lsl.end());
}

BusSpec::WireInfo::WireInfo(const std::string& coded_label)
{
    if (coded_label.find("i:") == 0)
    {
        _label = coded_label.substr(2);
        _single_value_type = BusSpec::SingleValueType::Int;
    }
    else if (coded_label.find("b:") == 0)
    {
        _label = coded_label.substr(2);
        _single_value_type = BusSpec::SingleValueType::Bool;
    }
    else if (coded_label.find("f:") == 0)
    {
        _label = coded_label.substr(2);
        _single_value_type = BusSpec::SingleValueType::Scalar;
    }
    else
    {
        _label = coded_label;
        _single_value_type = BusSpec::SingleValueType::Scalar;
    }
}

SignalId BusInfo::operator[](const std::string& label) const
{
    pooya_trace("label: " + label);
    auto pos = label.find(".");
    if (pos == std::string::npos)
        return operator[](static_cast<const BusSpec&>(_spec).index_of(label)).second;

    auto sig = operator[](static_cast<const BusSpec&>(_spec).index_of(label.substr(0, pos))).second;
    pooya_verify_bus(sig);
    return sig->as_bus()->operator[](label.substr(pos + 1));
}

SignalId BusInfo::at(const std::string& label) const
{
    pooya_trace("label: " + label);
    auto pos = label.find(".");
    auto index = static_cast<const BusSpec&>(_spec).index_of(pos == std::string::npos ? label : label.substr(0, pos));
    pooya_verify(index < static_cast<const BusSpec&>(_spec)._wires.size(), label + ": label not found in bus!");
    auto sig = at(index).second;
    if (pos == std::string::npos)
        return sig;

    pooya_verify_bus(sig);
    return sig->as_bus()->at(label.substr(pos + 1));
}

void ValuesArray::init(std::size_t num_values, std::size_t num_state_variables)
{
    pooya_trace("ValuesArray: " + std::to_string(num_values) + " " + std::to_string(num_state_variables) + " states");
    pooya_verify(num_state_variables <= num_values, "Number of state variables (" + std::to_string(num_state_variables)
        + " cannot be more than the number of values (" + std::to_string(num_values) + ").");

    _values.resize(num_values);
    new (&_state_variables) StateVariables(_values.data(), num_state_variables);
    _state_variable_derivs.resize(num_state_variables);
}

void BusInfo::_set(std::size_t index, SignalId sig)
{
    pooya_trace("index: " + std::to_string(index));
    pooya_verify(index < _signals.size(), "bus wire index out of range!");
    auto& ns = _signals[index];
    pooya_verify(ns.second == nullptr, ns.first + ": cannot re-assign a bus wire!");
#if !defined(NDEBUG)
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
#endif // !defined(NDEBUG)
    ns.second = sig;
}

}
