
#include <iostream>
#include <algorithm>
#include <experimental/random>

#include "blocks.hpp"

// todo:
// - define and implement an interface for input and output values
// - define inputs and outputs for Submodel block

namespace blocks
{

std::string Node::_make_valid_given_name(const std::string& given_name) const
{
    std::string ret(given_name);
    if (given_name.empty())
    {
        std::cout << "Warning: node name cannot be empty.";
        ret = Base::generate_random_name();
        std::cout << " Proceeding with the random name \"" << ret << "\".\n";
    }
    else if (given_name.find_first_of("/. ") != std::string::npos)
    {
        std::cout << "Warning: the node name \"" << given_name << "\" contains invalid characters.";
        ret = Base::generate_random_name();
        std::cout << " Proceeding with the random name \"" << ret << "\" instead.\n";
    }
    return ret;
}

bool Node::set_owner(Submodel& owner)
{
    assert(_name.empty() && (_id == NoId));
    if (!_name.empty() || (_id != NoId)) return false;

    auto* model = owner.get_model();
    if (!model) return false;

    std::string reg_name = owner.make_node_name(_given_name);
    _id = model->get_or_register_node(reg_name);
    if (_id == NoId)
        return false;

    _name = reg_name;
    return true;
}

NodeValues::NodeValues(const std::initializer_list<std::pair<Node, Value>>& list, Submodel& owner)
{
    for (const auto& v: list)
    {
        auto node(v.first);
        node.set_owner(owner);
        insert_or_assign(node, v.second);
    }
}

Base::Base(Submodel* parent, std::string given_name, const Nodes& iports, const Nodes& oports/*, bool register_oports*/) :
    _parent(parent)
{
    _assign_valid_given_name(given_name);
    _name = _parent ? (_parent->name() + "/" + _given_name) : ("/" + _given_name);

    if (parent)
    {
        parent->add_component(*this);

        _iports.reserve(iports.size());
        for (const auto& p: iports)
        {
            _iports.push_back(p);
            if (p.id() == Node::NoId) _iports.back().set_owner(*parent);
        }

        _oports.reserve(oports.size());
        for (auto& p: oports)
        {
            _oports.push_back(p);
            if (p.id() == Node::NoId) _oports.back().set_owner(*parent);
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
//     if (_update_node_ids)
//     {
//         auto* model = get_model();
//         if (model)
//             for (auto& node: _raw_names)
//                 model->register_node(node);
//         _update_node_ids = false;
//     }
//     // assert(x.first.size() == _raw_names.size());
//     NodeIdValues ret;
//     // ret.first.reserve(_raw_names.size());
//     // ret.second.reserve(_raw_names.size());
//     // auto p = x.second.begin();
//     for (const auto& node: _raw_names)
//     {
//         ret.insert_or_assign(node, x.at(node));
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

std::string Submodel::make_node_name(const std::string& given_name)
{
    return _name + "." + given_name;
}

Node Submodel::create_node(const std::string& given_name)
{
    Node node(given_name);
    node.set_owner(*this);
    return node;
}

uint Submodel::_process(double t, Values& values, bool reset)
{
    if (reset)
        _processed = false;

    uint n_processed = 0;
    if (not _processed)
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
        for (auto* component: _components)
            if (!component->traverse(cb, level + 1, max_level))
                return false;

    return true;
}

Model::Model(std::string name) : Submodel(nullptr, name)
{
}

Node::Id Model::get_or_register_node(const std::string& name)
{
    assert(!name.empty());
    if (name.empty()) return Node::NoId;

    Node::Id ret;

    auto it = std::find(_registered_nodes.begin(), _registered_nodes.end(), name);
    if (it == _registered_nodes.end())
    {
        ret = _registered_nodes.size();
        _registered_nodes.push_back(name);
    }
    else
    {
        ret = std::distance(_registered_nodes.begin(), it);
    }

    return ret;
}

}
