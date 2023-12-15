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
        if (!values.valid(signal))
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

template<>
void SinT<double>::activation_function(double /*t*/, Values& values)
{
    scalar_output(0, std::sin(scalar_input(0)));
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
    _assign_valid_given_name(_given_name);
    _full_name = "/" + _given_name;
}

bool Model::init(Parent& parent, const Signals& iports, const Signals& oports)
{
    return true;
}

}
