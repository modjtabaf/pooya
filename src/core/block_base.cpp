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
#include <iostream>

#include "block_base.hpp"

namespace pooya
{

bool Block::init(Parent& parent, const Signals& iports, const Signals& oports)
{
    assert(_parent == nullptr);
    if (_parent) return false;

    _parent = &parent;
    verify(_parent->is_initialized(), _given_name + ": parent block is not initialized yet!");

    _assign_valid_given_name(_given_name);
    if (_full_name.empty())
        _full_name = _parent ? (_parent->full_name() + "/" + _given_name) : ("/" + _given_name);

    verify((_num_iports == NoIOLimit) || (iports.size() == _num_iports),
        _num_iports == 0 ?
            _full_name + " cannot take any input." :
            _full_name + " requires " + std::to_string(_num_iports) + std::string(" input signal") + (_num_iports == 1 ? "." : "s."));

    verify((_num_oports == NoIOLimit) || (oports.size() == _num_oports),
        _num_oports == 0 ?
            _full_name + " cannot generate any output." :
            _full_name + " requires " + std::to_string(_num_oports) + std::string(" output signal") + (_num_oports == 1 ? "." : "s."));
    
    _iports.reserve(iports.size());
    _dependencies.reserve(iports.size());
    for (const auto& p: iports)
    {
        _iports.push_back(p);
        _add_dependecny(p);
    }
    _dependencies.shrink_to_fit();

    _oports.reserve(oports.size());
    for (auto& p: oports)
        _oports.push_back(p);
    
    _initialized = true;
    return true;
}

bool Block::_add_dependecny(Signal signal)
{
    if (std::find(_dependencies.begin(), _dependencies.end(), signal) == _dependencies.end())
    {
        _dependencies.push_back(signal);
        return true;
    }

    return false;
}

void Block::_assign_valid_given_name(std::string given_name)
{
    auto verify_unique_name_cb = [&] (const Block& c, uint32_t level) -> bool
    {
        return (level == 0) || (c._given_name != given_name);
    };

    if (given_name.empty())
    {
        given_name = "unnamed_block";
    }
    else
    {
        std::replace(given_name.begin(), given_name.end(), ' ', '_');
        std::replace(given_name.begin(), given_name.end(), '~', '_');
        std::replace(given_name.begin(), given_name.end(), '/', '_');
    }

    _given_name = given_name;

    if (_parent)
    {
        uint n = 0;
        while (!_parent->traverse(verify_unique_name_cb, 0, 1))
            given_name = _given_name + "_" + std::to_string(n++);

        _given_name = given_name;
    }
}

void Block::_mark_unprocessed()
{
    _processed = false;
}

uint Block::_process(double t, Values& values, bool /*go_deep*/)
{
    if (_processed)
        return 0;

    for (auto& signal: _dependencies)
    {
        if (!signal->as_bus() && !values.valid(signal))
            return 0;
    }

    activation_function(t, values);

    _processed = true;
    return 1;
}

Model* Block::model()
{
    return _parent ? _parent->model() : nullptr;
}

Model::~Model()
{
    for (auto* si: _signal_infos)
        delete si;
}

Signal Model::lookup_signal(const std::string& name, bool exact_match) const
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

template<>
typename Types<double>::Signal Model::register_signal<double>(const std::string& name)
{
    if (name.empty()) return nullptr;

    verify(!lookup_signal(name, true), "Re-registering a signal is not allowed!");

    auto index = _signal_infos.size();
    auto* sig = new ScalarSignalInfo(name, index, _vi_index++);
    _signal_infos.push_back(sig);

    return sig;
}

template<>
typename Types<int>::Signal Model::register_signal<int>(const std::string& name)
{
    if (name.empty()) return nullptr;

    verify(!lookup_signal(name, true), "Re-registering a signal is not allowed!");

    auto index = _signal_infos.size();
    auto* sig = new IntegerSignalInfo(name, index, _vi_index++);
    _signal_infos.push_back(sig);

    return sig;
}

ArraySignal Model::register_signal(const std::string& name, std::size_t size)
{
    if (name.empty()) return nullptr;

    verify(!lookup_signal(name, true), "Re-registering a signal is not allowed!");

    auto index = _signal_infos.size();
    auto* sig = new ArraySignalInfo(name, index, _vi_index++, size);
    _signal_infos.push_back(sig);

    return sig;
}

ValueSignalInfo* Model::_register_state(Signal sig, Signal deriv_sig)
{
    verify_float_signal(sig);
    verify_float_signal(deriv_sig);
    verify(!sig->_value->is_state(), sig->_full_name + ": signal is already registered as a state!");
    verify(!deriv_sig->_value->_is_deriv, deriv_sig->_full_name + ": signal is already registered as a state derivative!");
    verify((sig->_scalar && deriv_sig->_scalar) || (sig->_array && deriv_sig->_array && sig->_array->_size == deriv_sig->_array->_size),
        sig->_full_name + ", " + deriv_sig->_full_name + ": type or size mismatch!");

    ValueSignalInfo* ret = _signal_infos[sig->_index]->_value;
    ret->_deriv_sig = deriv_sig->_value;
    _signal_infos[deriv_sig->_index]->_value->_is_deriv = true;

    return ret;
}

std::string Parent::make_signal_name(const std::string& given_name, bool make_new)
{
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
    if (!make_new)
        return sig_name;

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
    Block::_mark_unprocessed();

    for (auto component: _components)
    {
        component->_mark_unprocessed();
    }
}

uint Parent::_process(double t, Values& values, bool go_deep)
{
    uint n_processed = 0;
    if (!_processed)
    {
        _processed = true;
        if (go_deep)
            for (auto* component: _components)
            {
                n_processed += component->_process(t, values);
                if (not component->processed())
                    _processed = false;
            }
    }

    return n_processed;
}

bool Parent::traverse(TraverseCallback cb, uint32_t level, decltype(level) max_level)
{
    if (level > max_level)
        return true;

    if (!Block::traverse(cb, level, max_level))
        return false;

    if (level < max_level)
    {
        for (auto& c: _components)
        {
            if (!c->traverse(cb, level + 1, max_level))
                return false;
        }
    }

    return true;
}

Signal Parent::get_generic_signal(const std::string& given_name)
{
    return model_ref().lookup_signal(make_signal_name(given_name), true);
}

template<>
typename Types<double>::Signal Parent::get_signal<double>(const std::string& given_name)
{
    Signal sig = get_generic_signal(given_name);
    if (!sig)
        return nullptr;
    verify_scalar_signal(sig);
    return sig->as_scalar();
}

template<>
typename Types<int>::Signal Parent::get_signal<int>(const std::string& given_name)
{
    Signal sig = get_generic_signal(given_name);
    if (!sig)
        return nullptr;
    verify_integer_signal(sig);
    return sig->as_integer();
}

ArraySignal Parent::get_signal(const std::string& given_name, std::size_t size)
{
    Signal sig = get_generic_signal(given_name);
    if (!sig)
        return nullptr;
    verify_array_signal_size(sig, size);
    return sig->as_array();
}

BusSignal Parent::get_bus(const std::string& given_name, const BusSpec& spec)
{
    Signal sig = get_generic_signal(given_name);
    if (!sig)
        return nullptr;
    verify_bus_signal_spec(sig, spec);
    return sig->as_bus();
}

ArraySignal Parent::signal(const std::string& given_name, std::size_t size)
{
    auto sig = get_signal(given_name, size);
    return sig ? sig : create_signal(given_name, size);
}

BusSignal Parent::bus(const std::string& given_name, const BusSpec& spec)
{
    auto sig = get_bus(given_name, spec);
    return sig ? sig : create_bus(given_name, spec);
}

template<>
typename Types<double>::Signal Parent::create_signal<double>(const std::string& given_name)
{
    return model_ref().register_signal<double>(make_signal_name(given_name, true))->as_scalar();
}

template<>
typename Types<int>::Signal Parent::create_signal<int>(const std::string& given_name)
{
    return model_ref().register_signal<int>(make_signal_name(given_name, true))->as_integer();
}

ArraySignal Parent::create_signal(const std::string& given_name, std::size_t size)
{
    return model_ref().register_signal(make_signal_name(given_name, true), size)->as_array();
}

template<typename Iter>
BusSignal Parent::create_bus(const std::string& given_name, const BusSpec& spec, Iter begin_, Iter end_)
{
    auto size = spec._wires.size();
    verify(std::distance(begin_, end_) <= int(size), "Too many entries in the initializer list!");

    std::vector<BusSignalInfo::LabelSignal> label_signals;
    label_signals.reserve(size);
    for (const auto& wi: spec._wires)
        label_signals.push_back({wi._label, nullptr});
    for (auto it=begin_; it != end_; it++)
    {
        auto index = spec.index_of(it->first);
        verify(!label_signals.at(index).second, std::string("Duplicate label: ") + it->first);
        label_signals.at(index).second = it->second;
    }
    auto wit = spec._wires.begin();
    for (auto& ls: label_signals)
    {
        if (!ls.second)
        {
            std::string name = given_name + "." + wit->_label;
            ls.second = wit->_scalar ? create_signal(name) : (wit->_array_size > 0 ? Signal(create_signal(name, wit->_array_size)) :
                create_bus(name, *wit->_bus));
        }
        wit++;
    }

    return model_ref().register_bus(make_signal_name(given_name, true), spec, label_signals.begin(), label_signals.end());
}

BusSignal Parent::create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<BusSignalInfo::LabelSignal>& l)
{
    verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");
    return create_bus(given_name, spec, l.begin(), l.end());
}

BusSignal Parent::create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<Signal>& l)
{
    verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");

    std::vector<BusSignalInfo::LabelSignal> label_signals;
    label_signals.reserve(l.size());

    auto wit = spec._wires.begin();
    for (const auto& sig: l)
    {
        label_signals.push_back({wit->_label, sig});
    }

    return create_bus(given_name, spec, label_signals.begin(), label_signals.end());
}

Signal Parent::clone_signal(const std::string& given_name, Signal sig)
{
    verify_valid_signal(sig);
    if (sig->as_scalar())
    {
        return create_signal(given_name);
    }
    else if (sig->as_array())
    {
        return create_signal(given_name, sig->as_array()->_size);
    }
    else
    {
        verify_bus_signal(sig);
        BusSignal bus = sig->as_bus();
        std::size_t n = bus->spec()._wires.size();
        std::vector<BusSignalInfo::LabelSignal> sigs;
        sigs.reserve(n);
        for (std::size_t k = 0; k < n; k++)
        {
            const auto& ns = bus->at(k);
            sigs.emplace_back(BusSignalInfo::LabelSignal(ns.first, clone_signal("", ns.second)));
        }
        return create_bus(given_name, bus->spec(), sigs.begin(), sigs.end());
    }
}

Model::Model(std::string given_name) : Parent(given_name, 0, 0)
{
    _assign_valid_given_name(_given_name);
    _full_name = "/" + _given_name;
    _initialized = true;
}

bool Model::init(Parent& parent, const Signals& iports, const Signals& oports)
{
    return true;
}

BusSignal Model::register_bus(const std::string& name, const BusSpec& spec, BusSignalInfo::LabelSignals::const_iterator begin_, BusSignalInfo::LabelSignals::const_iterator end_)
{
    if (name.empty()) return nullptr;

    verify(!lookup_signal(name, true), "Re-registering a signal is not allowed!");

    auto index = _signal_infos.size();
    auto* sig = new BusSignalInfo(name, index, spec, begin_, end_);
    _signal_infos.push_back(sig);

    return sig;
}

}
