
#include <iostream>
#include <algorithm>
#include <experimental/random>

#include "blocks.hpp"

// todo:
// - define and implement an interface for input and output values
// - define inputs and outputs for Submodel block

namespace blocks
{

std::string Signal::_make_valid_given_name(const std::string& given_name) const
{
    std::string ret(given_name);
    if (given_name.empty())
    {
        std::cout << "Warning: signal name cannot be empty.";
        ret = Base::generate_random_name();
        std::cout << " Proceeding with the random name \"" << ret << "\".\n";
    }
    else if (given_name.find_first_of("/. ") != std::string::npos)
    {
        std::cout << "Warning: the signal name \"" << given_name << "\" contains invalid characters.";
        ret = Base::generate_random_name();
        std::cout << " Proceeding with the random name \"" << ret << "\" instead.\n";
    }
    return ret;
}

bool Signal::set_owner(Submodel& owner)
{
    assert(_full_name.empty() && (_id == NoId));
    if (!_full_name.empty() || (_id != NoId)) return false;

    auto* model = owner.get_model();
    if (!model) return false;

    std::string reg_name = owner.make_signal_name(_given_name);
    _id = model->find_or_register_signal(reg_name);
    if (_id == NoId)
        return false;

    _full_name = reg_name;
    return true;
}

Base::Base(Submodel* parent, std::string given_name, const Signals& iports, const Signals& oports/*, bool register_oports*/) :
    _parent(parent)
{
    _assign_valid_given_name(given_name);
    _full_name = _parent ? (_parent->full_name() + "/" + _given_name) : ("/" + _given_name);

    if (parent)
    {
        parent->add_component(*this);

        _iports.reserve(iports.size());
        for (const auto& p: iports)
        {
            _iports.push_back(p);
            if (p.id() == Signal::NoId) _iports.back().set_owner(*parent);
        }

        _oports.reserve(oports.size());
        for (auto& p: oports)
        {
            _oports.push_back(p);
            if (p.id() == Signal::NoId) _oports.back().set_owner(*parent);
        }
    }
    else
    {
        _iports = iports;
        _oports = oports;
    }

    // auto* model = get_model();
    // if (model)
    // {
    //     if (register_oports)
    //     {
    //         for (auto& port: _oports)
    //         {
    //             if (std::find(_all_oports.cbegin(), _all_oports.cend(), port) != _all_oports.cend())
    //                 std::cout << *std::find(_all_oports.cbegin(), _all_oports.cend(), port) << "\n";
    //             assert(std::find(_all_oports.cbegin(), _all_oports.cend(), port) == _all_oports.cend());
    //             _all_oports.push_back(port);
    //         }
    //     }

    //     for (auto& port: _iports)
    //         model->register_iport(port, *this);
    // }
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

uint Base::_process(double t, Values& values, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    for (auto& iport: _iports)
    {
        if (!values.get(iport))
            return 0;
    }

    activation_function(t, values);

    _processed = true;
    return 1;
}

Model* Base::get_model()
{
    return _parent ? _parent->get_model() : nullptr;
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

uint Integrator::_process(double /*t*/, Values& values, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    _processed = values.get(_iports.front());
    return _processed ? 1 : 0; // is it safe to simply return _processed?
}

uint Memory::_process(double /*t*/, Values& values, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    values.set(_oports.front().id(), _value);
    _processed = true;
    return 1;
}

std::string Submodel::make_signal_name(const std::string& given_name)
{
    return _full_name + "." + given_name;
}

Signal Submodel::signal(const std::string& given_name)
{
    Signal signal(given_name);
    signal.set_owner(*this);
    return signal;
}

Signal Submodel::parameter(const std::string& given_name)
{
    auto* model = get_model();
    assert(model);
    Signal signal(given_name);
    signal.set_owner(*model);
    return signal;
}

uint Submodel::_process(double t, Values& values, bool reset)
{
    if (reset)
        _processed = false;

    uint n_processed = 0;
    if (!_processed)
    {
        _processed = true;
        for (auto& component: _components)
        {
            n_processed += component->_process(t, values, reset);
            if (not component->processed())
                _processed = false;
        }
    }

    return n_processed;
}

bool Submodel::traverse(TraverseCallback cb, uint32_t level, decltype(level) max_level)
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

Model::Model(std::string name) : Submodel(nullptr, name)
{
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
