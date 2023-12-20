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

void Signal::_set_owner(Parent& owner)
{
    assert(_full_name.empty() && (_id == NoId));
    if (!_full_name.empty() || (_id != NoId)) return;

    auto* model = owner.model();
    if (!model) return;

    std::string reg_name = owner.make_signal_name(_given_name);
    _id = model->signal_registry().find_signal(reg_name, true);
    if (_id == NoId)
        _id = model->signal_registry().register_signal(reg_name, _size);
    if (_id == NoId) return;

    _full_name = reg_name;
}

Values::Values(const pooya::Model& model)
{
    const auto& signals = model.signal_registry().signals();

    std::size_t states_size{0};

    // find the total sizes of signals and states
    for (const auto& signal: signals)
    {
        auto size = std::max<std::size_t>(signal._size, 1);
        _total_size += size;
        if (signal.is_state())
            states_size += size;
    }

    _values.resize(_total_size);

    double* state_start = _values.data();
    double* other_start = state_start + states_size;

    for (const auto& signal: signals)
    {
        auto& start = signal.is_state() ? state_start : other_start;
        _value_infos.push_back({signal, start, signal._size});
        start += std::max<std::size_t>(signal._size, 1);
    }

    assert(state_start == _values.data() + states_size);
    assert(other_start == _values.data() + _total_size);

    new (&_states) decltype(_states)(_values.data(), states_size);

    _derivs.resize(states_size);
    double* deriv_start = _derivs.data();

    // state-specific steps

    for (const auto& signal: signals)
    {
        if (!signal.is_state())
            continue;

        auto& deriv_vi = get_value_info(signal._deriv_id);
        if (signal.is_scalar())
            deriv_vi._deriv_scalar = deriv_start;
        else
            new (&deriv_vi._deriv_array) Eigen::Map<Eigen::ArrayXd>(deriv_start, signal._size);

        deriv_start += std::max<std::size_t>(signal._size, 1);
    }

    for (const auto& signal: signals)
    {
        if (!signal.is_state())
            continue;

        auto& vi = get_value_info(signal._id);
        if (signal.is_scalar())
            _set(vi, signal._iv[0]);
        else
            _set(vi, signal._iv);
    }
}

void Values::set_states(const Eigen::ArrayXd& states)
{
    _states = states;
    for (ValueInfo& vi: _value_infos)
    {
        if (!vi._si.is_state())
            continue;

        assert(!vi.is_assigned());
        vi._assigned = true;

        if (vi._si._is_deriv)
        {
            if (vi._deriv_scalar)
                *vi._deriv_scalar = *vi._scalar;
            else
                vi._deriv_array = vi._array;
        }
    }
}

std::vector<SignalRegistry::SignalInfo>::const_iterator SignalRegistry::_find_signal(const std::string& name, bool exact_match) const
{
    if (name.empty())
        return _signal_infos.end();

    auto name_len = name.length();

    auto it = std::find_if(_signal_infos.begin(), _signal_infos.end(),
        [&] (const SignalRegistry::SignalInfo& sig_info) -> bool
        {
            if (exact_match)
                return sig_info._full_name == name;
                
            auto str_len = sig_info._full_name.length();
            return (str_len >= name_len) && (sig_info._full_name.substr(str_len - name_len) == name);
        });

    return it;
}

Signal::Id SignalRegistry::find_signal(const std::string& name, bool exact_match) const
{
    auto sig_info = _find_signal(name, exact_match);
    if (sig_info == _signal_infos.end()) return Signal::NoId;

    return std::distance(_signal_infos.begin(), sig_info);
}

Signal::Id SignalRegistry::register_signal(const std::string& name, std::size_t size)
{
    if (name.empty()) return Signal::NoId;

    Signal::Id id = find_signal(name, true);
    verify(id == Signal::NoId, "Re-registering a signal is not allowed!");

    id = _signal_infos.size();
    _signal_infos.push_back({name, id, size});

    return id;
}

SignalRegistry::SignalInfo& SignalRegistry::_register_state(Signal::Id id, Signal::Id deriv_id)
{
    verify(id != Signal::NoId, "invalid state id!");
    verify(deriv_id != Signal::NoId, "invalid derivative id!");

    auto& si = _signal_infos[id];

    verify(!si.is_state(), si._full_name + ": signal is already registered as a state!");
    si._deriv_id = deriv_id;

    auto& deriv_si = _signal_infos[deriv_id];
    verify(!deriv_si._is_deriv, si._full_name + ": signal is already registered as a state derivative!");
    deriv_si._is_deriv = true;

    return si;
}

}
