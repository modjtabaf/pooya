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
#include "block.hpp"

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
        std::replace(given_name.begin(), given_name.end(), '.', '_');
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

template<>
void SinT<double>::activation_function(double /*t*/, Values& values)
{
    values.set<double>(_oports[0], std::sin(values.get<double>(_iports[0])));
}

std::string Parent::make_signal_name(const std::string& given_name)
{
    std::string name(given_name);
    if (name.empty())
    {
        name = "unnamed_signal_" + std::to_string(_unnamed_signal_counter++);
    }
    else
    {
        std::replace(name.begin(), name.end(), ' ', '_');
        std::replace(name.begin(), name.end(), '.', '_');
        std::replace(name.begin(), name.end(), '/', '_');
    }

    return _full_name + "." + name;
}

void Parent::_mark_unprocessed()
{
    Block::_mark_unprocessed();

    for (auto* component: _components)
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

ScalarSignal Parent::signal(const std::string& given_name)
{
    auto* model_ = model();
    if (!model_) return nullptr;

    std::string reg_name = make_signal_name(given_name);
    Signal sig = model_->signal_registry().find_signal(reg_name, true);
    if (!sig)
        sig = model_->signal_registry().register_signal(reg_name);

    verify_scalar_signal(sig);
    return sig->as_scalar();
}

ArraySignal Parent::signal(const std::string& given_name, std::size_t size)
{
    auto* model_ = model();
    if (!model_) return nullptr;

    std::string reg_name = make_signal_name(given_name);
    Signal sig = model_->signal_registry().find_signal(reg_name, true);
    if (!sig)
        sig = model_->signal_registry().register_signal(reg_name, size);

    verify_array_signal(sig);
    return sig->as_array();
}

template<typename Iter>
BusSignal Parent::signal(const std::string& given_name, const BusSpec& spec, Iter begin_, Iter end_)
{
    auto* model_ = model();
    if (!model_) return nullptr;

    std::string reg_name = make_signal_name(given_name);
    Signal sig = model_->signal_registry().find_signal(reg_name, true);
    if (!sig)
        sig = model_->signal_registry().register_signal(reg_name, spec, begin_, end_);

    verify_bus_signal(sig);
    return sig->as_bus();
}

BusSignal Parent::signal(const std::string& given_name, const BusSpec& spec, const std::initializer_list<BusSignalInfo::NameSignal>& l)
{
    return signal(given_name, spec, l.begin(), l.end());
}

BusSignal Parent::signal(const std::string& given_name, const BusSpec& spec, const std::initializer_list<Signal>& l)
{
    return signal(given_name, spec, l.begin(), l.end());
}

BusSignal Parent::signal(const std::string& given_name, const BusSpec& spec)
{
    Signals signals;
    signals.reserve(spec._wires.size());
    for (const auto& wi: spec._wires)
    {
        std::string name = given_name + "~" + wi._name;
        signals.push_back(wi._scalar ? Signal(signal(name)) : (wi._array_size > 0 ? Signal(signal(name, wi._array_size)) :
            Signal(signal(name, *wi._bus))));
    }
    return signal(given_name, spec, signals.begin(), signals.end());
}

Signal Parent::clone_signal(const std::string& given_name, Signal sig)
{
    verify_valid_signal(sig);
    if (sig->as_scalar())
    {
        verify_scalar_signal(sig);
        return signal(given_name);
    }
    else if (sig->as_array())
    {
        verify_array_signal(sig);
        return signal(given_name, sig->as_array()->_size);
    }
    else
    {
        verify_bus_signal(sig);
        BusSignal bus = sig->as_bus();
        std::size_t n = bus->spec()._wires.size();
        std::vector<BusSignalInfo::NameSignal> sigs;
        sigs.reserve(n);
        for (std::size_t k = 0; k < n; k++)
        {
            const auto& ns = bus->at(k);
            sigs.emplace_back(BusSignalInfo::NameSignal(ns.first, clone_signal("", ns.second)));
        }
        return signal(given_name, bus->spec(), sigs.begin(), sigs.end());
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

BusBlockBuilder::~BusBlockBuilder()
{
    for (auto* p: _blocks)
        if (p)
            delete p;
}

bool BusBlockBuilder::init(Parent& parent, const Signals& iports, const Signals& oports)
{
    if (!Block::init(parent, iports, oports))
        return false;

    iports.bind(0, _x);
    oports.bind(0, _y);

    verify(_x->spec() == _y->spec(), "Bus specs don't match!");

    return true;
}

void BusBlockBuilder::post_init()
{
    const auto& bus_spec = _x->spec();
    _blocks.reserve(bus_spec.total_size());
    traverse_bus("", bus_spec);
}

void BusBlockBuilder::traverse_bus(const std::string& path, const BusSpec& bus_spec)
{
    for (const auto& wi: bus_spec._wires)
    {
        auto new_path = path + "/" + wi._name;
        if (wi._bus)
            traverse_bus(new_path, *wi._bus);
        else
            block_builder(new_path, wi, _x->at(wi._name), _y->at(wi._name));
    }
}

}
