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

std::string Signal::_make_valid_given_name(const std::string& given_name) const
{
    std::string ret(given_name);
    if (given_name.empty())
    {
        ret = generate_random_name();
    }
    else if (given_name.find_first_of("/. ") != std::string::npos)
    {
        std::cout << "Warning: the signal name \"" << given_name << "\" contains invalid characters.";
        ret = generate_random_name();
        std::cout << " Proceeding with the random name \"" << ret << "\" instead.\n";
    }
    return ret;
}

void Signal::_set_owner(Parent& owner, std::size_t size)
{
    assert(!_si);
    if (_si) return;

    auto* model = owner.model();
    if (!model) return;

    std::string reg_name = owner.make_signal_name(_given_name);
    _si = model->signal_registry().find_signal(reg_name, true);
    verify(!_si || _si->_size == size, _given_name + ": size mismatch");
    if (!_si)
        _si = model->signal_registry().register_signal(reg_name, size);
}

Values::Values(const pooya::Model& model)
{
    const auto& signals = model.signal_registry().signals();

    std::size_t states_size{0};

    // find the total sizes of signals and states
    for (const auto* signal: signals)
    {
        auto size = std::max<std::size_t>(signal->_size, 1);
        _total_size += size;
        if (signal->is_state())
            states_size += size;
    }

    _values.resize(_total_size);

    double* state_start = _values.data();
    double* other_start = state_start + states_size;

    for (const auto* signal: signals)
    {
        auto& start = signal->is_state() ? state_start : other_start;
        _value_infos.push_back({*signal, start, signal->_size});
        start += std::max<std::size_t>(signal->_size, 1);
    }

    assert(state_start == _values.data() + states_size);
    assert(other_start == _values.data() + _total_size);

    new (&_states) decltype(_states)(_values.data(), states_size);

    _derivs.resize(states_size);
    double* deriv_start = _derivs.data();

    // state-specific steps

    for (const auto* signal: signals)
    {
        if (!signal->is_state())
            continue;

        auto& deriv_vi = get_value_info(signal->_deriv_si);
        if (signal->is_scalar())
            deriv_vi._deriv_scalar = deriv_start;
        else
            new (&deriv_vi._deriv_array) Eigen::Map<Eigen::ArrayXd>(deriv_start, signal->_size);

        deriv_start += std::max<std::size_t>(signal->_size, 1);
    }

    for (const auto* signal: signals)
    {
        if (!signal->is_state())
            continue;

        auto& vi = get_value_info(signal);
        if (signal->is_scalar())
            _set(vi, signal->_iv[0]);
        else
            _set(vi, signal->_iv);
    }
}

void Values::set_states(const Eigen::ArrayXd& states)
{
    _states = states;
    for (auto& vi: _value_infos)
    {
        if (!(vi._assigned = vi._si.is_state()))
            continue;

        if (vi._si._is_deriv)
        {
            if (vi._deriv_scalar)
                *vi._deriv_scalar = *vi._scalar;
            else
                vi._deriv_array = vi._array;
        }
    }
}

const SignalInfo* SignalRegistry::find_signal(const std::string& name, bool exact_match) const
{
    if (name.empty())
        return nullptr;

    auto name_len = name.length();

    auto it = std::find_if(_signal_infos.begin(), _signal_infos.end(),
        [&] (const SignalInfo* si) -> bool
        {
            if (exact_match)
                return si->_full_name == name;
                
            auto str_len = si->_full_name.length();
            return (str_len >= name_len) && (si->_full_name.substr(str_len - name_len) == name);
        });

    return it == _signal_infos.end() ? nullptr : *it;
}

SignalInfo* SignalRegistry::register_signal(const std::string& name, std::size_t size)
{
    if (name.empty()) return nullptr;

    const SignalInfo* si = find_signal(name, true);
    verify(!si, "Re-registering a signal is not allowed!");

    auto index = _signal_infos.size();
    auto* si_new = new SignalInfo(name, index, size);
    _signal_infos.push_back(si_new);

    return si_new;
}

SignalInfo* SignalRegistry::_register_state(const SignalInfo* si, const SignalInfo* deriv_si)
{
    verify(si, "invalid state info!");
    verify(deriv_si, "invalid derivative info!");

    // auto& si = _signal_infos[id];

    verify(!si->is_state(), si->_full_name + ": signal is already registered as a state!");

    // auto& deriv_si = _signal_infos[deriv_id];
    verify(!deriv_si->_is_deriv, si->_full_name + ": signal is already registered as a state derivative!");

    auto* ret = _signal_infos[si->_index];
    ret->_deriv_si = deriv_si;
    _signal_infos[deriv_si->_index]->_is_deriv = true;

    return ret;
}

}
