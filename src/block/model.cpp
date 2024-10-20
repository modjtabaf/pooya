/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "src/helper/util.hpp"
#include "src/signal/signal_id.hpp"
#include "src/signal/float_signal.hpp"
#include "src/signal/scalar_signal.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "model.hpp"

namespace pooya
{

SignalId Model::lookup_signal(const std::string& name, bool exact_match) const
{
    pooya_trace("block: " + full_name());
    if (name.empty()) {return SignalId();}

    auto name_len = name.length();

    auto it = std::find_if(_signal_infos.begin(), _signal_infos.end(),
        [&] (SignalId sig) -> bool
        {
            if (exact_match) {return sig->_full_name == name;}
                
            auto str_len = sig->_full_name.length();
            return (str_len >= name_len) && (sig->_full_name.substr(str_len - name_len) == name);
        });

    return it == _signal_infos.end() ? SignalId() : *it;
}

void Model::register_state_variable(FloatSignalId sig, FloatSignalId deriv_sig)
{
    pooya_trace("block: " + full_name());
    pooya_verify_signals_not_locked();
    pooya_verify_valid_signal(sig);
    pooya_verify_valid_signal(deriv_sig);
    pooya_verify(!sig->is_state_variable(), sig->_full_name + ": signal is already registered as a state variable!");
    pooya_verify(!deriv_sig->_is_deriv, deriv_sig->_full_name + ": signal is already registered as a state variable derivative!");
    pooya_verify((sig->is_scalar() && deriv_sig->is_scalar()) || (sig->is_array() && deriv_sig->is_array() && sig->as_array()._size == deriv_sig->as_array()._size),
        sig->_full_name + ", " + deriv_sig->_full_name + ": type or size mismatch!");

    _signal_infos[sig->_index]->as_float()._deriv_sig = deriv_sig;
    _signal_infos[deriv_sig->_index]->as_float()._is_deriv = true;
}

Model::Model(const std::string& given_name) : Parent(given_name, 0, 0)
{
    pooya_trace("block: " + full_name());
    _given_name = make_valid_given_name(_given_name);
    _full_name = "/" + _given_name;
    _initialized = true;
}

bool Model::init(Parent& /*parent*/, BusId /*ibus*/, BusId /*obus*/)
{
    return true;
}

void Model::lock_signals()
{
    pooya_trace("model: " + full_name());

    if (_signals_locked) {return;} // nothing to do!

    std::size_t state_variables_size{0};
    std::size_t total_size{0};

    // find the total sizes of signals and state variables
    for (const auto& signal: _signal_infos)
    {
        if (!signal->is_value()) {continue;}
        auto size = (signal->_type & (SignalInfo::ScalarType | SignalInfo::IntType | SignalInfo::BoolType)) ? 1 : signal->as_array()._size;
        total_size += size;
        if (signal->is_float() && signal->as_float().is_state_variable())
            {state_variables_size += size;}
    }

    _values.init(total_size, state_variables_size);

    double* state_variables_start = _values.values().data();
    double* other_start = state_variables_start + state_variables_size;

    for (auto& signal: _signal_infos)
    {
        if (!signal->is_value()) {continue;}
        auto& start = signal->is_float() && signal->as_float().is_state_variable() ? state_variables_start : other_start;
        auto size = (signal->_type & (SignalInfo::ScalarType | SignalInfo::IntType | SignalInfo::BoolType)) ? 0 : signal->as_array()._size;
        if (signal->is_scalar())
            {signal->as_scalar()._scalar_value = *start;}
        else if (signal->is_int())
            {signal->as_int()._int_value = *start;}
        else if (signal->is_bool())
            {signal->as_bool()._bool_value = *start;}
        else
            {new (&signal->as_array()._array_value) MappedArray(start, size);}
        start += std::max<std::size_t>(size, 1);
    }

    assert(state_variables_start == _values.values().data() + state_variables_size);
    assert(other_start == _values.values().data() + total_size);

    double* deriv_start = _values.state_variable_derivs().data();

    // state-variable-specific steps

    for (auto& signal: _signal_infos)
    {
        if (!signal->is_float() || !signal->as_float().is_state_variable()) {continue;}

        if (signal->is_scalar())
            {signal->as_scalar()._deriv_sig->as_scalar()._deriv_scalar_value = *deriv_start;}
        else
            {new (&signal->as_array()._deriv_sig->as_array()._deriv_array_value) MappedArray(deriv_start, signal->as_array()._size);}

        deriv_start += signal->is_scalar() ? 1 : signal->as_array()._size;
    }

    _signals_locked = true;
}

void Model::reset_with_state_variables(const Array& state_variables)
{
    pooya_trace0;
#ifdef POOYA_DEBUG
    _values._values.setZero();
    _values._state_variable_derivs.setZero();
#endif // defined(POOYA_DEBUG)
    pooya_verify(_values._state_variables.size() == state_variables.size(), "State variables size mismatch!");
    _values._state_variables = state_variables;
    for (auto& sig: _signal_infos)
    {
        if (!sig->is_value()) {continue;} // skip non-values

        if (sig->is_float()) // skip non-floats
        {
            sig->as_float()._assigned = bool(sig->as_float()._deriv_sig);
            if (!sig->as_float()._assigned) {continue;} // skip non-state-variables

            if (sig->as_float()._is_deriv) // skip non-derivatives
            {
                if (sig->is_scalar())
                    {sig->as_scalar()._deriv_scalar_value.value().get() = sig->as_scalar()._scalar_value.value().get();}
                else
                    {sig->as_array()._deriv_array_value = sig->as_array()._array_value;}
            }
        }
        else
        {
            sig->as_value()._assigned = false;
        }
    }
}

void Model::invalidate()
{
#ifdef POOYA_DEBUG
    _values._values.setZero();
    _values._state_variable_derivs.setZero();
#endif // defined(POOYA_DEBUG)
    for (auto& sig: _signal_infos)
    {
        if (sig->is_value())
        {
            sig->as_value()._assigned = false;
        }
    }
}

}
