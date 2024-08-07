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

#include <cassert>

#include "block_base.hpp"
#include "signal.hpp"
#include "util.hpp"

namespace pooya
{

bool Block::init(Parent& parent, BusId ibus, BusId obus)
{
    pooya_trace(_given_name);

    assert(!_parent);
    if (_parent) {return false;}

#if defined(POOYA_USE_SMART_PTRS)
    _parent = parent;
#else // defined(POOYA_USE_SMART_PTRS)
    _parent = &parent;
#endif // defined(POOYA_USE_SMART_PTRS)

    pooya_verify(parent.is_initialized(), _given_name + ": parent block is not initialized yet!");

    _given_name = util::make_valid_given_name(_given_name, &parent);
    if (_full_name.empty())
    {
        _full_name = parent.full_name() + "/" + _given_name;
    }

    pooya_verify((_num_iports == NoIOLimit) || (!ibus && _num_iports == 0) || (ibus && ibus->size() == _num_iports),
        _num_iports == 0 ?
            _full_name + " cannot take any input." :
            _full_name + " requires " + std::to_string(_num_iports) + std::string(" input signal") + (_num_iports == 1 ? "." : "s."));

    pooya_verify((_num_oports == NoIOLimit) || (!obus && _num_oports == 0) || (obus && obus->size() == _num_oports),
        _num_oports == 0 ?
            _full_name + " cannot generate any output." :
            _full_name + " requires " + std::to_string(_num_oports) + std::string(" output signal") + (_num_oports == 1 ? "." : "s."));
    
    if (ibus && ibus->size() > 0)
    {
        _dependencies.reserve(ibus->size());
        for (auto sig: *ibus)
        {
            if (sig.second->is_value())
            {
                add_dependency(sig.second->as_value());
            }
        }
        _dependencies.shrink_to_fit();
    }

    _ibus = ibus;
    _obus = obus;

    _initialized = true;
    return true;
}

bool Block::add_dependency(ValueSignalId sig)
{
    pooya_trace("block: " + full_name());
    pooya_verify_valid_signal(sig);
    if (std::find(_dependencies.begin(), _dependencies.end(), sig) == _dependencies.end())
    {
        _dependencies.push_back(sig);
        return true;
    }

    return false;
}

bool Block::remove_dependency(ValueSignalId sig)
{
    pooya_trace("block: " + full_name());
    pooya_verify_valid_signal(sig);
    auto it = std::find(_dependencies.begin(), _dependencies.end(), sig);
    if (it == _dependencies.end())
    {
        return false;
    }
    _dependencies.erase(it);
    return true;
}

void Block::_mark_unprocessed()
{
    _processed = false;
}

uint Block::_process(double t, bool /*go_deep*/)
{
    pooya_trace("block: " + full_name());
    if (_processed) {return 0;}
    for (auto& sig: _dependencies)
    {
        if (!sig->is_assigned()) {return 0;}
    }

    activation_function(t);

    _processed = true;
    return 1;
}

Model* Block::model()
{
    pooya_trace("block: " + full_name());

    if (_parent)
    {
#if defined(POOYA_USE_SMART_PTRS)
        return _parent->get().model();
#else // defined(POOYA_USE_SMART_PTRS)
        return _parent->model();
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return nullptr;
}

Model::~Model()
{
    pooya_trace("block: " + full_name());
#if !defined(POOYA_USE_SMART_PTRS)
    for (auto* si: _signal_infos) {delete si;}
#endif // defined(POOYA_USE_SMART_PTRS)
}

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
    pooya_verify((sig->_scalar && deriv_sig->_scalar) || (sig->_array && deriv_sig->_array && sig->as_array()->_size == deriv_sig->as_array()->_size),
        sig->_full_name + ", " + deriv_sig->_full_name + ": type or size mismatch!");

    _signal_infos[sig->_index]->as_float()->_deriv_sig = deriv_sig->as_float();
    _signal_infos[deriv_sig->_index]->as_float()->_is_deriv = true;
}

std::string Parent::make_signal_name(const std::string& given_name, bool make_new)
{
    pooya_trace("block: " + full_name());
    std::string name(given_name);
    if (name.empty())
    {
        name = "unnamed_signal_" + std::to_string(_unnamed_signal_counter++);
    }
    else
    {
        std::replace(name.begin(), name.end(), ' ', '_');
        std::replace(name.begin(), name.end(), '~', '_');
        std::replace(name.begin(), name.end(), '/', '_');
    }

    std::string sig_name =  _full_name + "~" + name;
    if (!make_new) {return sig_name;}

    auto& model_ = model_ref();

    int k = 0;
    std::string postfix = "";
    while(model_.lookup_signal(sig_name + postfix, true))
    {
        postfix = "_" + std::to_string(k++);
    }
    return sig_name + postfix;
}

void Parent::_mark_unprocessed()
{
    pooya_trace("block: " + full_name());
    Block::_mark_unprocessed();

#if defined(POOYA_USE_SMART_PTRS)
    for (auto& component: _components)
    {
        component.get()._mark_unprocessed();
    }
#else // defined(POOYA_USE_SMART_PTRS)
    for (auto* component: _components)
    {
        component->_mark_unprocessed();
    }
#endif // defined(POOYA_USE_SMART_PTRS)
}

uint Parent::_process(double t, bool go_deep)
{
    pooya_trace("block: " + full_name());
    uint n_processed = 0;
    if (!_processed)
    {
        _processed = true;
        if (go_deep)
#if defined(POOYA_USE_SMART_PTRS)
            for (auto& component: _components)
            {
                n_processed += component.get()._process(t);
                if (not component.get().processed()) {_processed = false;}
            }
#else // defined(POOYA_USE_SMART_PTRS)
            for (auto* component: _components)
            {
                n_processed += component->_process(t);
                if (not component->processed()) {_processed = false;}
            }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return n_processed;
}

bool Parent::traverse(TraverseCallback cb, uint32_t level, decltype(level) max_level)
{
    pooya_trace("block: " + full_name());
    if (level > max_level) {return true;}

    if (!Block::traverse(cb, level, max_level)) {return false;}

    if (level < max_level)
    {
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& c: _components)
        {
            if (!c.get().traverse(cb, level + 1, max_level)) {return false;}
        }
#else // defined(POOYA_USE_SMART_PTRS)
        for (auto* c: _components)
        {
            if (!c->traverse(cb, level + 1, max_level)) {return false;}
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return true;
}

SignalId Parent::get_generic_signal(const std::string& given_name)
{
    pooya_trace("block: " + full_name());
    return model_ref().lookup_signal(make_signal_name(given_name), true);
}

template<typename Iter>
BusId Parent::create_bus(const std::string& given_name, const BusSpec& spec, Iter begin_, Iter end_)
{
    pooya_trace("block: " + full_name());
    auto size = spec._wires.size();
    pooya_verify(std::distance(begin_, end_) <= int(size), "Too many entries in the initializer list!");

    std::vector<LabelSignalId> label_signals;
    label_signals.reserve(size);
    for (const auto& wi: spec._wires) {label_signals.push_back({wi.label(), SignalId()});}
    for (auto it=begin_; it != end_; it++)
    {
        auto index = spec.index_of(it->first);
        pooya_verify(!label_signals.at(index).second, std::string("Duplicate label: ") + it->first);
        label_signals.at(index).second = it->second;
    }
    auto wit = spec._wires.begin();
    for (auto& ls: label_signals)
    {
        if (!ls.second)
        {
            std::string name = given_name + "." + wit->label();
            if (wit->_bus)
                {ls.second = create_bus(name, *wit->_bus);}
            else if (wit->_array_size > 0)
                {ls.second = create_array_signal(name, wit->_array_size);}
            else if (wit->single_value_type() == BusSpec::SingleValueType::Scalar)
                {ls.second = create_scalar_signal(name);}
            else if (wit->single_value_type() == BusSpec::SingleValueType::Int)
                {ls.second = create_int_signal(name);}
            else if (wit->single_value_type() == BusSpec::SingleValueType::Bool)
                {ls.second = create_bool_signal(name);}
            else
                {pooya_verify(false, name + ": unknown wire type!");}
        }
        wit++;
    }

    return model_ref().register_bus(make_signal_name(given_name, true), spec, label_signals.begin(), label_signals.end());
}

BusId Parent::create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<LabelSignalId>& l)
{
    pooya_trace("block: " + full_name());
    pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");
    return create_bus(given_name, spec, l.begin(), l.end());
}

BusId Parent::create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<SignalId>& l)
{
    pooya_trace("block: " + full_name());
    pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");

    LabelSignalIdList label_signals;
    label_signals.reserve(l.size());

    auto wit = spec._wires.begin();
    for (const auto& sig: l)
    {
        label_signals.push_back({wit->label(), sig});
        wit++;
    }

    return create_bus(given_name, spec, label_signals.begin(), label_signals.end());
}

SignalId Parent::clone_signal(const std::string& given_name, SignalId sig)
{
    pooya_trace("block: " + full_name());
    pooya_verify_valid_signal(sig);
    if (sig->is_scalar())
    {
        return create_scalar_signal(given_name);
    }
    else if (sig->is_array())
    {
        return create_array_signal(given_name, sig->as_array()->size());
    }
    else
    {
        pooya_verify_bus(sig);
        BusId bus = sig->as_bus();
        std::size_t n = bus->spec()._wires.size();
        std::vector<LabelSignalId> sigs;
        sigs.reserve(n);
        for (std::size_t k = 0; k < n; k++)
        {
            const auto& ns = bus->at(k);
            sigs.emplace_back(ns.first, clone_signal("", ns.second));
        }
        return create_bus(given_name, bus->spec(), sigs.begin(), sigs.end());
    }
}

bool Parent::add_block(Block& component, const LabelSignals& iports, const LabelSignals& oports)
{
    pooya_trace("block: " + full_name());

    auto make_bus = [&](const LabelSignals& ports) -> BusId
    {
        std::vector<BusSpec::WireInfo> wire_infos;
        LabelSignalIdList wires;
        for (const auto& ls: ports)
        {
            pooya_verify_valid_signal(ls.second);
            if (ls.second->is_scalar())
                {wire_infos.emplace_back(ls.first);}
            else if (ls.second->is_int())
                {wire_infos.emplace_back("i:" + ls.first);}
            else if (ls.second->is_bool())
                {wire_infos.emplace_back("b:" + ls.first);}
            else if (ls.second->is_array())
                {wire_infos.emplace_back(ls.first, ls.second->as_array()->size());}
            else if (ls.second->is_bus())
                {wire_infos.emplace_back(ls.first, ls.second->as_bus()->_spec);}
            else
                {pooya_verify(false, "unknown signal type!");}
            
            wires.emplace_back(ls.first, ls.second);
        }
        _interface_bus_specs.emplace_back(std::make_unique<BusSpec>(wire_infos.begin(), wire_infos.end()));
        return create_bus("", *_interface_bus_specs.back(), wires.begin(), wires.end());
    };

    if (!component.init(*this, make_bus(iports), make_bus(oports))) {return false;}

#if defined(POOYA_USE_SMART_PTRS)
    _components.push_back(component);
#else // defined(POOYA_USE_SMART_PTRS)
    _components.push_back(&component);
#endif // defined(POOYA_USE_SMART_PTRS)
    component.post_init();
    return true;
}

Model::Model(const std::string& given_name) : Parent(given_name, 0, 0)
{
    pooya_trace("block: " + full_name());
    _given_name = util::make_valid_given_name(_given_name);
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
        auto size = (signal->is_scalar() || signal->is_int() || signal->is_bool()) ? 1 : signal->as_array()->_size;
        total_size += size;
        if (signal->is_float() && signal->as_float()->is_state_variable())
            {state_variables_size += size;}
    }

    _values.init(total_size, state_variables_size);

    double* state_variables_start = _values.values().data();
    double* other_start = state_variables_start + state_variables_size;

    for (auto& signal: _signal_infos)
    {
        if (!signal->is_value()) {continue;}
        auto& start = signal->_float && signal->as_float()->is_state_variable() ? state_variables_start : other_start;
        auto size = (signal->_scalar || signal->_int || signal->_bool) ? 0 : signal->as_array()->_size;
        if (signal->_scalar)
#if defined(POOYA_USE_SMART_PTRS)
            {signal->as_scalar()->_scalar_value = *start;}
#else // defined(POOYA_USE_SMART_PTRS)
            {signal->as_scalar()->_scalar_value = start;}
#endif // defined(POOYA_USE_SMART_PTRS)
        else if (signal->_int)
#if defined(POOYA_USE_SMART_PTRS)
            {signal->as_int()->_int_value = *start;}
#else // defined(POOYA_USE_SMART_PTRS)
            {signal->as_int()->_int_value = start;}
#endif // defined(POOYA_USE_SMART_PTRS)
        else if (signal->_bool)
#if defined(POOYA_USE_SMART_PTRS)
            {signal->as_bool()->_bool_value = *start;}
#else // defined(POOYA_USE_SMART_PTRS)
            {signal->as_bool()->_bool_value = start;}
#endif // defined(POOYA_USE_SMART_PTRS)
        else
            {new (&signal->as_array()->_array_value) MappedArray(start, size);}
        start += std::max<std::size_t>(size, 1);
    }

    assert(state_variables_start == _values.values().data() + state_variables_size);
    assert(other_start == _values.values().data() + total_size);

    double* deriv_start = _values.state_variable_derivs().data();

    // state-variable-specific steps

    for (auto& signal: _signal_infos)
    {
        if (!signal->_float || !signal->as_float()->is_state_variable()) {continue;}

        if (signal->_scalar)
#if defined(POOYA_USE_SMART_PTRS)
            {signal->as_scalar()->_deriv_sig->as_scalar()->_deriv_scalar_value = *deriv_start;}
#else // defined(POOYA_USE_SMART_PTRS)
            {signal->as_scalar()->_deriv_sig->as_scalar()->_deriv_scalar_value = deriv_start;}
#endif // defined(POOYA_USE_SMART_PTRS)
        else
            {new (&signal->as_array()->_deriv_sig->as_array()->_deriv_array_value) MappedArray(deriv_start, signal->as_array()->_size);}

        deriv_start += signal->is_scalar() ? 1 : signal->as_array()->_size;
    }

    _signals_locked = true;
}

void Model::reset_with_state_variables(const Array& state_variables)
{
    pooya_trace0;
#ifndef POOYA_NDEBUG
    _values._values.setZero();
    _values._state_variable_derivs.setZero();
#endif // !defined(POOYA_NDEBUG)
    pooya_verify(_values._state_variables.size() == state_variables.size(), "State variables size mismatch!");
    _values._state_variables = state_variables;
    for (auto& sig: _signal_infos)
    {
        if (!sig->_value) {continue;} // skip non-values

        if (sig->_float) // skip non-floats
        {
            sig->as_float()->_assigned = bool(sig->as_float()->_deriv_sig);
            if (!sig->as_float()->_assigned) {continue;} // skip non-state-variables

            if (sig->as_float()->_is_deriv) // skip non-derivatives
            {
                if (sig->_scalar)
#if defined(POOYA_USE_SMART_PTRS)
                    {sig->as_scalar()->_deriv_scalar_value.value().get() = sig->as_scalar()->_scalar_value.value().get();}
#else // defined(POOYA_USE_SMART_PTRS)
                    {*sig->as_scalar()->_deriv_scalar_value = *sig->as_scalar()->_scalar_value;}
#endif // defined(POOYA_USE_SMART_PTRS)
                else
                    {sig->as_array()->_deriv_array_value = sig->as_array()->_array_value;}
            }
        }
        else
        {
            sig->as_value()->_assigned = false;
        }
    }
}

void Model::invalidate()
{
#ifndef POOYA_NDEBUG
    _values._values.setZero();
    _values._state_variable_derivs.setZero();
#endif // !defined(POOYA_NDEBUG)
    for (auto& sig: _signal_infos)
    {
        if (sig->_value)
        {
            sig->as_value()->_assigned = false;
        }
    }
}

}
