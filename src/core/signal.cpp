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
#include <iostream>
#include <cassert>
#include <string>
#include <utility>

#include "signal.hpp"
#include "pooya.hpp"

namespace pooya
{

void LabelSignals::_init(LabelSignalIdList::const_iterator begin_, LabelSignalIdList::const_iterator end_)
{
    pooya_trace0;
    for (auto it=begin_; it != end_; it++)
        push_back(*it);
}

LabelSignals::LabelSignals(SignalId signal)
{
    pooya_trace0;
    LabelSignalIdList lsl({{_make_auto_label(0), signal}});
    _init(lsl.begin(), lsl.end());
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

// std::ostream& operator<<(std::ostream& os, const LabelSignals& signals)
// {
//     pooya_trace0;
//     os << "signals:\n";
//     for (const auto& ls: signals)
//         os << "- " << ls.first << ": " << ls.second << "\n";
//     return os;
// }

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
        return operator[](_spec.index_of(label)).second;

    auto sig = operator[](_spec.index_of(label.substr(0, pos))).second;
    verify_bus_signal(sig);
    return sig->as_bus()->operator[](label.substr(pos + 1));
}

SignalId BusInfo::at(const std::string& label) const
{
    pooya_trace("label: " + label);
    auto pos = label.find(".");
    if (pos == std::string::npos)
        return at(_spec.index_of(label)).second;

    auto sig = at(_spec.index_of(label.substr(0, pos))).second;
    verify_bus_signal(sig);
    return sig->as_bus()->at(label.substr(pos + 1));
}

Values::Values(const pooya::Model& model)
{
    pooya_trace("model: " + model.full_name());
    const auto& signals = model.signals();

    std::size_t states_size{0};

    // find the total sizes of signals and states
    for (const auto* signal: signals)
    {
        if (!signal->as_value()) continue;
        auto size = (signal->as_scalar() || signal->as_int() || signal->as_bool()) ? 1 : signal->as_array()->_size;
        _total_size += size;
        if (signal->as_value()->is_state())
            states_size += size;
    }

    _values.resize(_total_size);

    double* state_start = _values.data();
    double* other_start = state_start + states_size;

    for (const auto* signal: signals)
    {
        if (!signal->as_value()) continue;
        auto& start = signal->as_value()->is_state() ? state_start : other_start;
        auto size = (signal->as_scalar() || signal->as_int() || signal->as_bool()) ? 0 : signal->as_array()->_size;
        if (signal->as_int())
            _value_infos.push_back({*signal->as_int(), start});
        else if (signal->as_bool())
            _value_infos.push_back({*signal->as_bool(), start});
        else
            _value_infos.push_back({*signal->as_value(), start, size});
        start += std::max<std::size_t>(size, 1);
    }

    assert(state_start == _values.data() + states_size);
    assert(other_start == _values.data() + _total_size);

    new (&_states) decltype(_states)(_values.data(), states_size);

    _derivs.resize(states_size);
    double* deriv_start = _derivs.data();

    // state-specific steps

    for (const auto* signal: signals)
    {
        if (!signal->as_value()) continue;
        if (!signal->as_value()->is_state())
            continue;

        auto& deriv_vi = get_value_info(signal->as_value()->deriv_info());
        if (signal->as_scalar())
            deriv_vi._deriv_scalar = deriv_start;
        else
            new (&deriv_vi._deriv_array) Eigen::Map<Eigen::ArrayXd>(deriv_start, signal->as_array()->_size);

        deriv_start += signal->as_scalar() ? 1 : signal->as_array()->_size;
    }

    for (const auto* signal: signals)
    {
        if (!signal->as_value()) continue;
        if (!signal->as_value()->is_state())
            continue;

        if (signal->as_scalar())
            set<double>(signal, signal->as_scalar()->iv());
        else
            set<Array>(signal, signal->as_array()->iv());
    }
}

#ifndef POOYA_NDEBUG
const decltype(Values::_states)& Values::states() const
{
    for (const auto& vi: _value_infos)
    {
        verify(!vi._si->is_state() || vi.is_assigned(), "unassigned state");
    }
    return _states;
}
#endif // !definded(POOYA_NDEBUG)

void Values::invalidate()
{
#ifndef POOYA_NDEBUG
    _values.setZero();
    _derivs.setZero();
#endif // !defined(POOYA_NDEBUG)
    for (auto& vi: _value_infos)
    {
        vi._assigned = false;
    }
}

void Values::reset_with_states(const Eigen::ArrayXd& states)
{
    pooya_trace0;
#ifndef POOYA_NDEBUG
    _values.setZero();
    _derivs.setZero();
#endif // !defined(POOYA_NDEBUG)
    _states = states;
    for (auto& vi: _value_infos)
    {
        vi._assigned = vi._si->is_state();
        if (!vi._assigned)
            continue;

        if (vi._si->is_deriv())
        {
            if (vi._deriv_scalar)
                *vi._deriv_scalar = *vi._scalar;
            else
                vi._deriv_array = vi._array;
        }
    }
}

void BusInfo::_set(std::size_t index, SignalId sig)
{
    pooya_trace("index: " + std::to_string(index));
    verify(index < _signals.size(), "bus wire index out of range!");
    auto& ns = _signals[index];
    verify(ns.second == nullptr, ns.first + ": cannot re-assign a bus wire!");
#if !defined(NDEBUG)
    auto& wi = _spec._wires[index];
    if (wi._bus)
    {
        verify_bus_signal_spec(sig, *wi._bus);
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Scalar)
    {
        verify_scalar_signal(sig);
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Int)
    {
        verify_int_signal(sig);
    }
    else if (wi.single_value_type() == BusSpec::SingleValueType::Bool)
    {
        verify_bool_signal(sig);
    }
    else
    {
        verify_array_signal_size(sig, wi._array_size);
    }
#endif // !defined(NDEBUG)
    ns.second = sig;
}

}
