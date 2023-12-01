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
#include <algorithm>
#include <experimental/random>

#include "pooya.hpp"

namespace pooya
{

std::string Signal::_make_valid_given_name(const std::string& given_name) const
{
    std::string ret(given_name);
    if (given_name.empty())
    {
        ret = Base::generate_random_name();
    }
    else if (given_name.find_first_of("/. ") != std::string::npos)
    {
        std::cout << "Warning: the signal name \"" << given_name << "\" contains invalid characters.";
        ret = Base::generate_random_name();
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
    _id = model->find_or_register_signal(reg_name);
    if (_id == NoId) return;

    _full_name = reg_name;
}

bool Base::init(Parent& parent, const Signals& iports, const Signals& oports)
{
    assert(_parent == nullptr);
    if (_parent) return false;

    _parent = &parent;

    _assign_valid_given_name(_given_name);
    _full_name = _parent ? (_parent->full_name() + "/" + _given_name) : ("/" + _given_name);

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
    
    return true;
}

bool Base::_add_dependecny(const Signal& signal)
{
    if (std::find_if(_dependencies.begin(), _dependencies.end(),
        [&] (const Signal& s) -> bool
        {
            return s.full_name() == signal.full_name();
        }) == _dependencies.end())
    {
        _dependencies.push_back(signal);
        return true;
    }

    return false;
}

void Base::_assign_valid_given_name(std::string given_name)
{
    auto verify_unique_name_cb = [&] (const Base& c, uint32_t /*level*/) -> bool
    {
        return (&c == static_cast<Base*>(_parent)) || (c._given_name != given_name);
    };

    if (given_name.empty())
    {
        given_name = generate_random_name();
        std::cout << "Warning: given_name cannot be empty. Proceeding with the random name \"" << given_name << "\" instead.\n";
    }
    else if (given_name.find_first_of("./ ") != std::string::npos)
    {
        std::cout << "Warning: the given name \"" << given_name << "\" contains invalid characters.";
        given_name = generate_random_name();
        std::cout << " Proceeding with the random name \"" << given_name << "\" instead.\n";
    }
    else if (_parent)
    {
        if (!_parent->traverse(verify_unique_name_cb, 0, 1))
        {
            std::cout << "Warning: the given name \"" << given_name << "\" is already in use.";
            given_name = generate_random_name();
            std::cout << " Proceeding with the random name \"" << given_name << "\" instead.\n";
        }
    }
    else
    {
        _given_name = given_name;
        return;
    }

    while (!_parent->traverse(verify_unique_name_cb, 0, false))
    {
        std::cout << "Warning: the given name \"" << given_name << "\" is already in use.";
        given_name = generate_random_name();
        std::cout << "Proceeding with the random name \"" << given_name << "\" instead.\n";
    }

    _given_name = given_name;
}

void Base::_mark_unprocessed()
{
    _processed = false;
}

uint Base::_process(double t, Values& values, bool /*go_deep*/)
{
    if (_processed)
        return 0;

    for (auto& signal: _dependencies)
    {
        if (!values.get(signal))
            return 0;
    }

    activation_function(t, values);

    _processed = true;
    return 1;
}

Model* Base::model()
{
    return _parent ? _parent->model() : nullptr;
}

std::string Base::generate_random_name(int len)
{
    std::string name;
    name.reserve(len);
    for (auto i = 0; i < len; i++)
        name += char(std::experimental::randint(int('a'), int('z')));
    return name;
}

// NodeIdValues Bus::activation_function(double /*t*/, const NodeIdValues& x)
// {
//     if (_update_signal_ids)
//     {
//         auto* model = get_model();
//         if (model)
//             for (auto& signal: _raw_names)
//                 model->register_signal(signal);
//         _update_signal_ids = false;
//     }
//     // assert(x.first.size() == _raw_names.size());
//     NodeIdValues ret;
//     // ret.first.reserve(_raw_names.size());
//     // ret.second.reserve(_raw_names.size());
//     // auto p = x.second.begin();
//     for (const auto& signal: _raw_names)
//     {
//         ret.insert_or_assign(signal, x.at(signal));
//     }
//     return ret;
// }

uint Integrator::_process(double /*t*/, Values& values, bool /*go_deep*/)
{
    if (_processed)
        return 0;

    _processed = values.get(_iports.front());
    return _processed ? 1 : 0; // is it safe to simply return _processed?
}

uint Memory::_process(double /*t*/, Values& values, bool /*go_deep*/)
{
    if (_processed)
        return 0;

    values.set(_oports.front().id(), _value);
    _processed = true;
    return 1;
}

std::string Parent::make_signal_name(const std::string& given_name)
{
    return _full_name + "." + given_name;
}

void Parent::_mark_unprocessed()
{
    Base::_mark_unprocessed();

    for (auto* component: _components)
        component->_mark_unprocessed();
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

    if (!Base::traverse(cb, level, max_level))
        return false;

    if (level < max_level)
        for (auto it = _components.begin(); it != _components.end(); it++)
            if (!(*it)->traverse(cb, level + 1, max_level))
                return false;

    return true;
}

Model::Model(std::string given_name) : Parent(given_name)
{
    init(*this);
}

bool Model::init(Parent& parent, const Signals& iports, const Signals& oports)
{
    assert(_parent == nullptr);
    assert(&parent == this);

    _assign_valid_given_name(_given_name);
    _full_name = "/" + _given_name;
    
    return true;
}

Signal::Id Model::find_or_register_signal(const std::string& name)
{
    assert(!name.empty());
    if (name.empty()) return Signal::NoId;

    Signal::Id ret;

    auto it = std::find(_registered_signals.begin(), _registered_signals.end(), name);
    if (it == _registered_signals.end())
    {
        ret = _registered_signals.size();
        _registered_signals.push_back(name);
    }
    else
    {
        ret = std::distance(_registered_signals.begin(), it);
    }

    return ret;
}

Signal::Id Model::find_signal(const std::string& name, bool exact_match) const
{
    assert(!name.empty());
    if (name.empty()) return Signal::NoId;

    auto name_len = name.length();

    auto it = exact_match ?
        std::find(_registered_signals.begin(), _registered_signals.end(), name) :
        std::find_if(_registered_signals.begin(), _registered_signals.end(),
            [&] (const std::string& str) -> bool
            {
                auto str_len = str.length();
                return ((str_len >= name_len) && (str.substr(str_len - name_len) == name));
            });

    return it == _registered_signals.end() ? Signal::NoId : std::distance(_registered_signals.begin(), it);
}

}
