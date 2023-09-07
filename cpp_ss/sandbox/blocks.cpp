
#include <iostream>
#include <algorithm>
#include <experimental/random>

#include "blocks.hpp"

namespace blocks
{

Node::Node(const std::string& str, Model& model) : std::string(str)
{
    model.register_node(*this);
}

Node::Node(const char* str, Model& model) : std::string(str)
{
    model.register_node(*this);
}

Node::Node(int n, Model& model) : std::string(std::to_string(n))
{
    model.register_node(*this);
}

Node::Node(Model& model) : std::string("-")
{
    model.register_node(*this);
}

NodeIdValues::NodeIdValues(const std::initializer_list<std::pair<Node, Value>>& list, Model& model)
{
    for (const auto& v: list)
    {
        auto node(v.first);
        model.register_node(node);
        insert_or_assign(node, v.second);
    }
}

Base::Base(Submodel *parent, const char* name, const Nodes& iports, const Nodes& oports/*, bool register_oports*/) :
    _parent(parent), _name(name)
{
    if (parent)
    {
        if (parent->name() != "")
            _name = parent->name() + "." + _name;

        parent->add_component(*this);

        _iports.reserve(iports.size());
        for (auto& p: iports)
            _iports.emplace_back(parent->register_node(p, false));

        _oports.reserve(oports.size());
        for (auto& p: oports)
            _oports.emplace_back(parent->register_node(p, true));
    }
    else
    {
        _iports = iports;
        _oports = oports;
    }

    _iport_ids.reserve(iports.size());
    for (auto& p: _iports)
        _iport_ids.emplace_back(p);

    _oport_ids.reserve(oports.size());
    for (auto& p: _oports)
        _oport_ids.emplace_back(p);


    auto* model = get_model();
    if (model)
    {
        // if (register_oports)
        // {
        //     for (auto& port: _oports)
        //     {
        //         if (std::find(_all_oports.cbegin(), _all_oports.cend(), port) != _all_oports.cend())
        //             std::cout << *std::find(_all_oports.cbegin(), _all_oports.cend(), port) << "\n";
        //         assert(std::find(_all_oports.cbegin(), _all_oports.cend(), port) == _all_oports.cend());
        //         _all_oports.push_back(port);
        //     }
        // }

        for (auto& port: _iports)
            model->register_iport(port, *this);
    }
}

uint Base::_process(double t, NodeIdValues& x, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    for (auto& iport: _iport_ids)
    {
        if (x.find(iport) == x.end())
            return 0;
    }

    for (auto& oport: _oport_ids)
        assert(x.find(oport) == x.end());

    Values input_values;
    input_values.reserve(_iport_ids.size());
    for (auto& iport: _iport_ids)
        input_values.push_back(x.at(iport));
    auto output_values = activation_function(t, NodeIdValues(_iport_ids, input_values));
    for (auto& nv: output_values)
        x.insert_or_assign(nv.first, nv.second);

    _processed = true;
    return 1;
}

Model* Base::get_model()
{
    return _parent ? _parent->get_model() : nullptr;
}

NodeIdValues Bus::activation_function(double /*t*/, const NodeIdValues& x)
{
    if (_update_node_ids)
    {
        auto* model = get_model();
        if (model)
            for (auto& node: _raw_names)
                model->register_node(node);
        _update_node_ids = false;
    }
    // assert(x.first.size() == _raw_names.size());
    NodeIdValues ret;
    // ret.first.reserve(_raw_names.size());
    // ret.second.reserve(_raw_names.size());
    // auto p = x.second.begin();
    for (const auto& node: _raw_names)
    {
        ret.insert_or_assign(node, x.at(node));
    }
    return ret;
}

uint Integrator::_process(double /*t*/, NodeIdValues& x, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    _processed = x.find(_iport_ids.front()) != x.end();
    return _processed ? 1 : 0; // is it safe to simply return _processed?
}

uint Memory::_process(double /*t*/, NodeIdValues& x, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    x.insert_or_assign(_oport_ids.front(), _value);
    _processed = true;
    return 1;
}

Node Submodel::register_node(const Node& node, bool makenew)
{
    if (node > 0)
        return node;

    Node ret = node.empty() ? Node("-") : node;
    auto* model = get_model();

    // don't alter global node names
    if (ret[0] != '-')
    {
        if (model)
            model->register_node(ret);
        return ret;
    }

    // an auto-generated local node name?
    bool auto_gen = false;
    if (ret == "-")
    {
        if (makenew)
        {
            constexpr auto N = 10;
            char name[N + 2] = "-";
            for (auto i = 1; i <= N; i++)
                name[i] = char(std::experimental::randint(int('a'), int('z')));
            name[N + 1] = 0;
            ret = name;
            auto_gen = true;
        }
        else
        {
            assert(!_auto_node_name.empty());
            ret = _auto_node_name;
            if (model)
                model->register_node(ret);
            return ret;
        }
    }

    // non-auto local node name
    if (not _name.empty())
    {
        ret = "-" + _name + "." + &ret[1];
    }

    if (auto_gen)
        _auto_node_name = ret;

    if (model)
        model->register_node(ret);

    return ret;
}

uint Submodel::_process(double t, NodeIdValues& x, bool reset)
{
    if (reset)
        _processed = false;

    uint n_processed = 0;
    if (not _processed)
    {
        _processed = true;
        for (auto& component: _components)
        {
            n_processed += component->_process(t, x, reset);
            if (not component->is_processed())
                _processed = false;
        }
    }

    return n_processed;
}

bool Submodel::traverse(TraverseCallback cb)
{
    for (auto& component: _components)
    {
        if (not component->traverse(cb))
            return false;
    }

    return Base::traverse(cb);
}

void Model::register_node(Node& node)
{
    if (node > 0)
        return;

    auto it = std::find(_nodes_with_id.begin(), _nodes_with_id.end(), node);
    if (it == _nodes_with_id.end())
    {
        node._id = _nodes_with_id.size();
        _nodes_with_id.push_back(node);
    }
    else
    {
        node._id = std::distance(_nodes_with_id.begin(), it);
    }
}

}
