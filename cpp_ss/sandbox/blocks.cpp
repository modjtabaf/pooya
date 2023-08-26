
#include <iostream>
#include <algorithm>
#include <experimental/random>

#include "blocks.hpp"

namespace blocks
{

Base::Base(Submodel *parent, const char* name, const Nodes& iports, const Nodes& oports, bool register_oports) :
    _name(name)
{
    if (parent)
    {
        if (parent->name() != "")
            _name = parent->name() + "." + _name;

        parent->add_component(*this);

        _iports.reserve(iports.size());
        for (auto& p: iports)
        {
            _iports.emplace_back(parent->get_node_name(p, false));
        }

        _oports.reserve(oports.size());
        for (auto& p: oports)
        {
            _oports.emplace_back(parent->get_node_name(p, true));
        }
    }
    else
    {
        _iports = iports;
        _oports = oports;
    }

    if (register_oports)
    {
        for (auto& port: _oports)
        {
            if (std::find(_all_oports.cbegin(), _all_oports.cend(), port) != _all_oports.cend())
                std::cout << *std::find(_all_oports.cbegin(), _all_oports.cend(), port) << "\n";
            assert(std::find(_all_oports.cbegin(), _all_oports.cend(), port) == _all_oports.cend());
            _all_oports.push_back(port);
        }
    }

    for (auto& port: _iports)
        if (std::find(_all_iports.cbegin(), _all_iports.cend(), port) == _all_iports.cend())
            _all_iports.push_back(port);
}

uint Base::_process(double t, NodeValues& x, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    for (auto& iport: _iports)
    {
        if (x.find(iport) == x.end())
            return 0;
    }

    _known_values = x;

    for (auto& oport: _oports)
        assert(x.find(oport) == x.end());

    Values input_values;
    input_values.reserve(_iports.size());
    for (auto& iport: _iports)
        input_values.push_back(x.at(iport));
    auto output_values = activation_function(t, NodeValues(_iports, input_values));
    for (auto& nv: output_values)
        x.insert_or_assign(nv.first, nv.second);

    _processed = true;
    return 1;
}

Nodes Base::_all_iports;
Nodes Base::_all_oports;

uint Integrator::_process(double /*t*/, NodeValues& x, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    _processed = x.find(_iports.front()) != x.end();
    return _processed ? 1 : 0; // is it safe to simply return _processed?
}

uint Memory::_process(double /*t*/, NodeValues& x, bool reset)
{
    if (reset)
        _processed = false;

    if (_processed)
        return 0;

    x.insert_or_assign(_oports.front(), _value);
    _processed = true;
    return 1;
}

Node Submodel::get_node_name(const Node& node, bool makenew)
{
    if (node.is_locked())
        return node;

    Node ret = node.empty() ? Node() : node;

    // don't alter global node names
    if (ret[0] != '-')
        return ret;

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
            assert(not _auto_node_name.empty());
            return _auto_node_name;
        }
    }

    // non-auto local node name
    if (not _name.empty())
    {
        ret = "-" + _name + "." + &ret[1];
    }

    if (auto_gen)
        _auto_node_name = ret;

    ret.lock();
    return ret;
}

uint Submodel::_process(double t, NodeValues& x, bool reset)
{
    if (reset)
        _processed = false;

    _known_values = x;

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

}
