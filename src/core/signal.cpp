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

Values::Values(const SignalRegistry& signal_registry, const StatesInfo& states_info)
{
    const auto& signals = signal_registry.signals();

    // find the total size of signals
    for (const auto& signal: signals)
        _total_size += signal.second == 0 ? 1 : signal.second;

    _values.resize(_total_size);

    std::vector<bool> is_state(signals.size());
    std::fill(is_state.begin(), is_state.end(), false);

    _states_size = states_info.value().size();

    // mark the states
    for (const auto& si: states_info)
    {
        assert(!is_state[si._id]); // detect duplicate entries in states
        is_state[si._id] = true;
    }

    double* state_start = _values.data();
    double* other_start = state_start + _states_size;

    Signal::Id id = 0;
    for (const auto& signal: signals)
    {
        auto& start = is_state[id] ? state_start : other_start;
        _value_infos.push_back({id, start, signal.second, is_state[id]});
        start += signal.second == 0 ? 1 : signal.second;
        id++;
    }

    assert(state_start == _values.data() + _states_size);
    assert(other_start == _values.data() + _total_size);

    new (&_states) decltype(_states)(_values.data(), _states_size);

    _derivs.resize(_states_size);
    double* deriv_start = _derivs.data();
    for (const auto& si: states_info)
    {
        auto& vi = get_value_info(si._deriv_id);
        assert(!vi._is_deriv);
        vi._is_deriv = true;
        if (vi._scalar)
            vi._deriv_scalar = deriv_start;
        else
            new (&vi._deriv_array) Eigen::Map<Eigen::ArrayXd>(deriv_start, vi._array.size());
        deriv_start += vi._scalar ? 1 : vi._array.size();
    }
}

void Values::set_states(const Eigen::ArrayXd& states)
{
    _states = states;
    for (ValueInfo& vi: _value_infos)
    {
        if (!vi._is_state)
            continue;

        assert(!vi._assigned);
        vi._assigned = true;

        if (vi._is_deriv)
        {
            if (vi._deriv_scalar)
                *vi._deriv_scalar = *vi._scalar;
            else
                vi._deriv_array = vi._array;
        }
    }
}

void StatesInfo::lock()
{
    verify(!_locked, "attempting to lock an already-lokced StatesInfo object!");
    _locked = true;

    std::size_t total_size = 0;
    for (auto& si: *this)
        total_size += si._scalar ? 1 : si._value.size();

    std::sort(_Parent::begin(), _Parent::end(),
        [] (const StateInfo& si1, const StateInfo& si2) -> bool
        {
            return si1._id < si2._id;
        });
   
    _value.resize(total_size);
    std::size_t start = 0;
    for (const auto& si: *this)
    {
        auto size = si._value.size();
        _value.segment(start, size) = si._value;
        start += size;
    }
    assert(start == total_size);
}

std::vector<SignalRegistry::SignalInfo>::const_iterator SignalRegistry::_find_signal(const std::string& name, bool exact_match) const
{
    if (name.empty()) return _signals.end();

    auto name_len = name.length();

    auto it = std::find_if(_signals.begin(), _signals.end(),
        [&] (const SignalRegistry::SignalInfo& sig_info) -> bool
        {
            if (exact_match)
                return sig_info.first == name;
                
            auto str_len = sig_info.first.length();
            return (str_len >= name_len) && (sig_info.first.substr(str_len - name_len) == name);
        });

    return it;
}

Signal::Id SignalRegistry::find_signal(const std::string& name, bool exact_match) const
{
    auto sig_info = _find_signal(name, exact_match);
    if (sig_info == _signals.end()) return Signal::NoId;

    return std::distance(_signals.begin(), sig_info);
}

Signal::Id SignalRegistry::register_signal(const std::string& name, std::size_t size)
{
    if (name.empty()) return Signal::NoId;

    Signal::Id ret = find_signal(name, true);
    verify(ret == Signal::NoId, "Re-registering a signal is not allowed!");

    ret = _signals.size();
    _signals.push_back(std::make_pair(name, size));

    return ret;
}

}
