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

void Signal::post_init(Parent* owner)
{
    std::cout << "42 (post_init):\n" << std::flush;
    // don't proceed if it is initialized already
    if (!_full_name.empty() || (_id != NoId)) return;

    std::cout << "46 (post_init):\n" << std::flush;
    // set the owner if it was not set before
    if (!_owner) _owner = owner;

    std::cout << "50 (post_init): " << _owner << "\n" << std::flush;
    if (_owner)
    {
        // std::cout << "53 (post_init): " << _owner->full_name() << "\n" << std::flush;
        std::cout << "54 (post_init): " << _owner->model() << "\n" << std::flush;
    }
    // don't proceed if either owner or model is not available
    if (!_owner || !_owner->model()) return;

    // owner and model are both defined
    // finalize the initialization

    std::cout << "57 (post_init):\n" << std::flush;
    std::string reg_name = _owner->make_signal_name(_given_name);
    _id = _owner->model()->find_or_register_signal(reg_name);
    if (_id == NoId) return;

    std::cout << "62 (post_init):\n" << std::flush;
    _full_name = reg_name;
    std::cout << "64 (post_init):\n" << std::flush;
}

Base::Base(std::string given_name, const Signals& iports, const Signals& oports/*, bool register_oports*/)
    : _given_name(given_name)
{
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
}

void Base::post_init(Parent* parent)
{
    // set the parent if it was not set before
    if (!_parent)
        _parent = parent;

    // don't proceed if either parent or model is not available
    if (!_parent || !_parent->model()) return;

    if (_full_name.empty())
    {
        _assign_valid_given_name(_given_name);
        _full_name = _parent->full_name() + "/" + _given_name;
    }

    auto* model = _parent->model();

    for (auto& p: _iports)
        p.post_init(model);

    for (auto& p: _oports)
        p.post_init(model);

    for (auto& p: _dependencies)
        p.post_init(model);
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
    // assert(_parent);
    std::cout << "193 (model): " << (_parent ? _parent->full_name() : "") << "\n" << std::flush;
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

    for (auto& component: _components)
        component.first._mark_unprocessed();
}

uint Parent::_process(double t, Values& values, bool go_deep)
{
    uint n_processed = 0;
    if (!_processed)
    {
        _processed = true;
        if (go_deep)
            for (auto& component: _components)
            {
                n_processed += component.first._process(t, values);
                if (not component.first.processed())
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
            if (!it->first.traverse(cb, level + 1, max_level))
                return false;

    return true;
}

Model::Model(std::string name) : Parent(name)
{
    _assign_valid_given_name(name);
    _full_name = "//" + _given_name;
}

void Model::post_init_model()
{
    auto post_init_cb = [&] (Base& c, uint32_t /*level*/) -> bool
    {
        c.post_init(this);
        return true;
    };

    traverse(post_init_cb, 0);
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
