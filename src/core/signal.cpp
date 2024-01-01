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

#include <iostream>
#include <cassert>
#include "signal.hpp"
#include "pooya.hpp"

namespace pooya
{

SignalRegistry::~SignalRegistry()
{
    for (auto* si: _signal_infos)
        delete si;
}

Values::Values(const pooya::Model& model)
{
    const auto& signals = model.signal_registry().signals();

    std::size_t states_size{0};

    // find the total sizes of signals and states
    for (const auto* signal: signals)
    {
        if (!signal->as_value()) continue;
        auto size = signal->as_scalar() ? 1 : signal->as_array()->_size;
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
        auto size = signal->as_scalar() ? 0 : signal->as_array()->_size;
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

        // auto& vi = get_value_info(signal->as_value());
        if (signal->as_scalar())
            set<double>(signal, signal->as_scalar()->iv());
        else
            set<Array>(signal, signal->as_array()->iv());
    }
}

void Values::set_states(const Eigen::ArrayXd& states)
{
    _states = states;
    for (auto& vi: _value_infos)
    {
        if (!(vi._assigned = vi._si.is_state()))
            continue;

        if (vi._si.is_deriv())
        {
            if (vi._deriv_scalar)
                *vi._deriv_scalar = *vi._scalar;
            else
                vi._deriv_array = vi._array;
        }
    }
}

Signal SignalRegistry::find_signal(const std::string& name, bool exact_match) const
{
    if (name.empty())
        return nullptr;

    auto name_len = name.length();

    auto it = std::find_if(_signal_infos.begin(), _signal_infos.end(),
        [&] (Signal sig) -> bool
        {
            if (exact_match)
                return sig->_full_name == name;
                
            auto str_len = sig->_full_name.length();
            return (str_len >= name_len) && (sig->_full_name.substr(str_len - name_len) == name);
        });

    return it == _signal_infos.end() ? nullptr : *it;
}

ScalarSignal SignalRegistry::register_signal(const std::string& name)
{
    if (name.empty()) return nullptr;

    verify(!find_signal(name, true), "Re-registering a signal is not allowed!");

    auto index = _signal_infos.size();
    auto* sig = new ScalarSignalInfo(name, index, _vi_index++);
    _signal_infos.push_back(sig);

    return sig;
}

ArraySignal SignalRegistry::register_signal(const std::string& name, std::size_t size)
{
    if (name.empty()) return nullptr;

    verify(!find_signal(name, true), "Re-registering a signal is not allowed!");

    auto index = _signal_infos.size();
    auto* sig = new ArraySignalInfo(name, index, _vi_index++, size);
    _signal_infos.push_back(sig);

    return sig;
}

ValueSignalInfo* SignalRegistry::_register_state(Signal sig, Signal deriv_sig)
{
    verify_value_signal(sig);
    verify_value_signal(deriv_sig);
    verify(!sig->_value->is_state(), sig->_full_name + ": signal is already registered as a state!");
    verify(!deriv_sig->_value->_is_deriv, deriv_sig->_full_name + ": signal is already registered as a state derivative!");
    verify((sig->_scalar && deriv_sig->_scalar) || (sig->_array && deriv_sig->_array && sig->_array->_size == deriv_sig->_array->_size),
        sig->_full_name + ", " + deriv_sig->_full_name + ": type or size mismatch!");

    ValueSignalInfo* ret = _signal_infos[sig->_index]->_value;
    ret->_deriv_sig = deriv_sig->_value;
    _signal_infos[deriv_sig->_index]->_value->_is_deriv = true;

    return ret;
}

}
