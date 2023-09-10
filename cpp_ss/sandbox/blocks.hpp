
#ifndef __BLOCKS_HPP__
#define __BLOCKS_HPP__

#include <algorithm>
#include <initializer_list>
#include <iterator>
#include <string>
#include <map>
#include <vector>
#include <cassert>

#include "../3rdparty/eigen/Eigen/Core"

using namespace Eigen;

namespace blocks
{

class Base;
class Submodel;
class Model;
class NodeIdValues;

class Value : public ArrayXd
{
public:
    using ArrayXd::ArrayXd;

    Value(double v=0) : ArrayXd(1)
    {
        (*this)[0] = v;
    }
};

class Node : public std::string
{
    friend class Model;

public:
    using Id = uint32_t;

protected:
    Id _id{0};

public:
    using Parent = std::string;

    Node(const std::string& str) : std::string(str) {}
    Node(const char* str) : std::string(str) {}
    Node(int n) : std::string(std::to_string(n)) {}
    Node() : std::string("-") {}
    Node(const std::string& str, Model& model);
    Node(const char* str, Model& model);
    Node(int n, Model& model);
    Node(Model& model);
    Node(const Node& node) = default;

    Node& operator=(const Node& rhs) = default;
    std::size_t operator()(const Node& node) const
    {
        return operator()(*static_cast<const Parent*>(&node));
    }
    operator Id() const
    {
        return _id;
    }
    Id id() const {return _id;}
};

class NodeIds : public std::vector<Node::Id>
{
public:
    using Parent = std::vector<Node::Id>;
    using Parent::vector;

    NodeIds(Node::Id node)
    {
        push_back(node);
    }
    NodeIds(const Node& node)
    {
        push_back(node);
    }
};

class Nodes : public std::vector<Node>
{
public:
    using Parent = std::vector<Node>;
    using Parent::vector;

    Nodes(const Node& node)
    {
        push_back(node);
    }
};

using Scalars          = std::vector<double>;
using Values           = std::vector<Value>;
using TraverseCallback = std::function<bool(const Base&, uint32_t level)>;
using ActFunction      = std::function<Value(double, const Value&)>;

std::ostream& operator<<(std::ostream&, const class NodeIdValues&);

class NodeIdValues : public std::unordered_map<Node::Id, Value>
{
protected:
    using Parent = std::unordered_map<Node::Id, Value>;

public:
    // using Parent::unordered_map;
    NodeIdValues() = default;
    NodeIdValues(const std::initializer_list<std::pair<Node, Value>>& list, Model& model);
    NodeIdValues(const std::initializer_list<std::pair<Node::Id, Value>>& list)
    {
        for (const auto& v: list)
            insert_or_assign(v.first, v.second);
    }
    NodeIdValues(const NodeIds& nodes, const Values& values)
    {
        auto value = values.begin();
        for (const auto& node: nodes)
        {
            insert_or_assign(node, *(value++));
        }
    }

    // NodeIds::const_iterator find(const Node& node) const
    // {
    //     assert(first.size() == second.size());
    //     return std::find(first.cbegin(), first.cend(), node);
    // }

    // const Value& at(const Node& node) const
    // {
    //     assert(first.size() == second.size());
    //     auto k = std::distance(first.begin(), find(node));
    //     return second.at(k);
    // }

    // void insert_or_assign(const Node& node, const Value& value)
    // {
    //     assert(first.size() == second.size());
    //     auto it = find(node);
    //     if (it == first.cend())
    //     {
    //         first.push_back(node);
    //         second.push_back(value);
    //     }
    //     else
    //     {
    //         auto k = std::distance(first.cbegin(), it);
    //         second.at(k) = value;
    //     }
    // }

    void join(const NodeIdValues& other)
    {
        // assert(first.size() == second.size());
        // auto node = other.first.begin();
        for(const auto& value: other)
            insert_or_assign(value.first, value.second);
    }
};

class States : public std::unordered_map<Node::Id, std::pair<Value, Node::Id>> // state, value, derivative
{
protected:
    using Pair = std::pair<Value, Node::Id>;
    using Parent = std::unordered_map<Node::Id, Pair>;

public:
    using Parent::unordered_map;

    void insert_or_assign(Node::Id state, const Value& value, Node::Id deriv)
    {
        Parent::insert_or_assign(state, Pair(value, deriv));
    }
};

inline std::ostream& operator<<(std::ostream& os, const NodeIdValues& nvs)
{
    // auto node = nv.first.begin();
    for (auto& nv: nvs)
    {
        os << "  - " << nv.first << ": " << nv.second << "\n";
        // node++;
    }
    return os << "\n";
}

class Base
{
protected:
    Nodes _iports;
    NodeIds _iport_ids;
    Nodes _oports;
    NodeIds _oport_ids;
    Submodel* const _parent;
    std::string _given_name;
    std::string _name;

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);

public:
    Base(Submodel* parent, std::string given_name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes()/*, bool register_oports=true*/);

    virtual void get_states(States& /*states*/) {}
    virtual void step(double /*t*/, const NodeIdValues& /*states*/) {}
    virtual NodeIdValues activation_function(double /*t*/, const NodeIdValues& /*x*/)
    {
        assert(false);
        return NodeIdValues();
    }
    virtual Model* get_model();

    bool processed() const {return _processed;}
    const std::string& given_name() const {return _given_name;}
    const std::string& name() const {return _name;}
    const Nodes& iports() const {return _iports;}
    const Nodes& oports() const {return _oports;}

    virtual uint _process(double t, NodeIdValues& x, bool reset);

    // def __repr__(self):
    //     return str(type(self)) + ":" + self._name + ", iports:" + str(self._iports) + ", oports:" + str(self._oports)

    virtual bool traverse(TraverseCallback cb, uint32_t level, bool /*go_deep=true*/)
    {
        return cb(*this, level);
    }

    static std::string generate_random_name(int len = 10);
}; // class Base

// class Value(Base):
//     def activation_function(self, t, x):
//         return [x[0]]

class Bus : public Base
{
protected:
    bool _update_node_ids{true};
    std::vector<Node> _raw_names;

public:
    // class BusValues : public Values
    // {
    // protected:
    //     Nodes _names;

    // public:
    //     BusValues(const Nodes& names, const Values& values) :
    //         Values(values), _names(names) {}
    
    //     // def __repr__(self):
    //     //     return 'BusValues(' + super().__repr__() + ')'
    // };

    Bus(Submodel* parent, std::string given_name, const Nodes& iports=Node(), const Node& oport=Node()) :
        Base(parent, given_name, iports, oport)
    {
        _raw_names.reserve(iports.size());
        for (const auto& p: iports)
            _raw_names.push_back(p);
    }

    NodeIdValues activation_function(double /*t*/, const NodeIdValues& x) override;
};

// class BusSelector(Base):
//     def __init__(self, name, iport='-', signals=[], oports=[]):
//         if not isinstance(oports, (list, tuple)):
//             oports = [oports]
//         if not isinstance(signals, (list, tuple)):
//             signals = [signals]
            
//         if not signals:
//             signals = oports
//         elif not oports:
//             oports = signals
//         assert len(signals) == len(oports)

//         super().__init__(name, iport, oports)
//         self._signals = signals
//         self._indices = None

//     def activation_function(self, t, x):
//         bus_values = x[0]
//         if self._indices is None:
//             self._indices = [bus_values._names.index(s)
//                              for s in self._signals]
//         return [bus_values[k] for k in self._indices]

class InitialValue : public Base
{
protected:
    Value _value;

public:
    InitialValue(Submodel* parent, std::string given_name, const Node& iport=Node(), const Node& oport=Node()) :
        Base(parent, given_name, iport, oport) {}

    NodeIdValues activation_function(double /*t*/, const NodeIdValues& x) override
    {
        if (_value.size() == 0)
        {
            _value = x.begin()->second;
            _iports.clear();
            _iport_ids.clear();
        }
        return NodeIdValues(_oport_ids.front(), {_value});
    }
};

class Const : public Base
{
protected:
    Value _value;

public:
    Const(Submodel* parent, std::string given_name, const Value& value, const Nodes& oport=Nodes({Node()})) :
        Base(parent, given_name, Nodes(), oport), _value(value) {}

    Const(Submodel* parent, std::string given_name, double value, const Nodes& oport={Node()}) :
        Base(parent, given_name, Nodes(), oport), _value(1)
    {
         _value << value;
    }

    NodeIdValues activation_function(double /*t*/, const NodeIdValues& /*x*/) override
    {
        return NodeIdValues(_oport_ids.front(), {_value});
    }
};

class Gain : public Base
{
protected:
    double _k;

public:
    Gain(Submodel* parent, std::string given_name, double k, const Nodes& iport=Nodes({Node()}), const Nodes& oport=Nodes({Node()})) :
        Base(parent, given_name, iport, oport), _k(k) {}

    NodeIdValues activation_function(double /*t*/, const NodeIdValues& x) override
    {
        return NodeIdValues(_oport_ids.front(), {_k * x.begin()->second[0]});
    }
};

class Sin : public Base
{
public:
    Sin(Submodel* parent, std::string given_name, const Nodes& iport=Nodes({Node()}), const Nodes& oport=Nodes({Node()})) :
        Base(parent, given_name, iport, oport) {}

    NodeIdValues activation_function(double /*t*/, const NodeIdValues& x) override
    {
        return NodeIdValues(_oport_ids.front(), {x.begin()->second.sin()});
    }
};

class Function : public Base
{
protected:
    ActFunction _act_func;

public:
    Function(Submodel* parent, std::string given_name, ActFunction act_func, const Node& iport=Node(), const Node& oport=Node()) :
        Base(parent, given_name, {iport}, {oport}), _act_func(act_func) {}

    NodeIdValues activation_function(double t, const NodeIdValues& x) override
    {
        return NodeIdValues(_oport_ids.front(), {_act_func(t, x.begin()->second[0])});
    }
};

// class MIMOFunction(Base):
//     def __init__(self, name, act_func, iports, oports):
//         super().__init__(name, iports=iports, oports=oports)
//         self._act_func = act_func

//     def activation_function(self, t, x):
//         return self._act_func(t, x)

class AddSub : public Base
{
protected:
    std::string _operators;
    double      _initial;

public:
    AddSub(Submodel* parent, std::string given_name, const char* operators, const Nodes& iports, const Node& oport=Node(), double initial=0.0) :
        Base(parent, given_name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    NodeIdValues activation_function(double /*t*/, const NodeIdValues& x) override
    {
        Value ret = Value::Constant(x.begin()->second.size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& node: _iport_ids)
        {
            const auto& v = x.at(node);
            if (*p == '+')
                ret += v;
            else if (*p == '-')
                ret -= v;
            else
                 assert(false);
            p++;
        }
        return NodeIdValues(_oport_ids.front(), {ret});
    }
};

class MulDiv : public Base
{
protected:
    std::string _operators;
    double      _initial;

public:
    MulDiv(Submodel* parent, std::string given_name, const char* operators, const Nodes& iports, const Node& oport=Node(), double initial=1.0) :
        Base(parent, given_name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    NodeIdValues activation_function(double /*t*/, const NodeIdValues& x) override
    {
        Value ret = Value::Constant(x.begin()->second.size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& node: _iport_ids)
        {
            const auto &v = x.at(node);
            if (*p == '*')
                ret *= v;
            else if (*p == '/')
                ret /= v;
            else
                 assert(false);
            p++;
        }
        return NodeIdValues(_oport_ids.front(), {ret});
    }
};

class Integrator : public Base
{
protected:
    double _value;

public:
    Integrator(Submodel* parent, std::string given_name, const Node& iport=Node(), const Node& oport=Node(), double ic=0.0) :
        Base(parent, given_name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void get_states(States& states) override
    {
        states.insert_or_assign(_oport_ids.front(), Value(_value), _iport_ids.front());
    }

    void step(double /*t*/, const NodeIdValues& states) override
    {
        _value = states.at(_oport_ids.front())[0];
    }

    uint _process(double t, NodeIdValues& x, bool reset) override;
};

// # it is still unclear how to deal with states when using this numerical integrator
// # class NumericalIntegrator(Base):
// #     def __init__(self, y0, name, iport='-', oport='-'):
// #         super().__init__(name, [iport], [oport])
// #         self._t = None
// #         self._x = None
// #         self._y = y0
    
// #     def step(self, t, states):
// #         self._t = t
// #         self._x = states[self._iports[0]]
// #         self._y = states[self._oports[0]]

// #     def activation_function(self, t, x):
// #         if self._t is None:
// #             self._t = t
// #             self._x = x[0]
// #             return [self._y]
// #         else:
// #             return [self._y + 0.5*(t - self._t)*(x[0] + self._x)]

class Delay : public Base
{
protected:
    double  _lifespan;
    Scalars _t;
    Values  _x;

public:
    Delay(Submodel* parent, std::string given_name, const Nodes& iports, const Nodes& oport=Nodes({Node()}), double lifespan=10.0) :
        Base(parent, given_name, iports, oport), _lifespan(lifespan) {}

    void step(double t, const NodeIdValues& states) override
    {
        if (not _t.empty())
        {
            double t1 = t - _lifespan;
            int k = 0;
            for (const auto& v: _t)
            {
                if (v >= t1)
                    break;
                k++;
            }
            _t.erase(_t.begin(), _t.begin() + k);
            _x.erase(_x.begin(), _x.begin() + k);
        }

        assert(_t.empty() or (t > _t.back()));
        _t.push_back(t);
        _x.push_back(states.at(_iport_ids.front()));
    }

    NodeIdValues activation_function(double t, const NodeIdValues& x) override
    {
        if (_t.empty())
            return NodeIdValues(_oport_ids.front(), {x.at(_iport_ids[2])[0]});
    
        const double delay = x.at(_iport_ids[1])[0];
        t -= delay;
        if (t <= _t.front())
        {
            return NodeIdValues(_oport_ids.front(), {x.at(_iport_ids[2])});
        }
        else if (t >= _t.back())
        {
            return NodeIdValues(_oport_ids.front(), {_x.back()});
        }

        int k = 0;
        for (const auto& v: _t)
        {
            if (v >= t)
                break;
            k++;
        }

        return NodeIdValues(_oport_ids.front(), {(_x[k][0] - _x[k - 1][0])*(t - _t[k - 1])/(_t[k] - _t[k - 1]) + _x[k - 1][0]});
    }
};

class Memory : public Base
{
protected:
    Value _value;

public:
    Memory(Submodel* parent, std::string given_name, const Node& iport=Node(), const Node& oport=Node(), const Value& ic=Value::Zero(1)) :
        Base(parent, given_name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void step(double /*t*/, const NodeIdValues& states) override
    {
        _value = states.at(_iport_ids.front());
    }

    // # Memory can be implemented either by defining the following activation function
    // #   (which is more straightforward) or through overloading the _process method
    // #   which is more efficient since it deosn't rely on the input signal being known.
    // #   Both approaches are supposed to lead to the exact same results.

    // # def activation_function(self, t, x):
    // #     return [self._value]

    uint _process(double t, NodeIdValues& x, bool reset) override;
};

class Derivative : public Base
{
protected:
    bool _first_step{true};
    double _t;
    Value  _x;
    Value  _y;

public:
    Derivative(Submodel* parent, std::string given_name, const Node& iport=Node(), const Node& oport=Node(), const Value& y0=Value::Zero(1)) :
        Base(parent, given_name, iport, oport), _y(y0) {}

    void step(double t, const NodeIdValues& states) override
    {
        _t = t;
        _x = states.at(_iport_ids.front());
        _y = states.at(_oport_ids.front());
        _first_step = false;
    }

    NodeIdValues activation_function(double t, const NodeIdValues& x) override
    {
        if (_first_step)
        {
            _t = t;
            _x = x.begin()->second;
            return NodeIdValues(_oport_ids.front(), {_y});
        }
        else if (_t == t)
            return NodeIdValues(_oport_ids.front(), {_y});

        return NodeIdValues(_oport_ids.front(), {((x.begin()->second - _x)/(t - _t))});
    }
};

class Submodel : public Base
{
protected:
    std::vector<Base*> _components;
    std::string _auto_node_name;

public:
    Submodel(Submodel* parent, std::string given_name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes()) :
        Base(parent, given_name, iports, oports/*, false*/) {}

    void add_component(Base& component)
    {
        _components.push_back(&component);
    }

    void get_states(States& states) override
    {
        for (auto* component: _components)
            component->get_states(states);
    }

    void step(double t, const NodeIdValues& states) override
    {
        for (auto& component: _components)
            component->step(t, states);
    }

    Node register_node(const Node& node, bool makenew);
    uint _process(double t, NodeIdValues& x, bool reset) override;
    bool traverse(TraverseCallback cb, uint32_t level, bool go_deep=true) override;

}; // class Submodel

class Model final : public Submodel
{
protected:
    using NodeBlocksMap = std::unordered_map<std::string, std::pair<bool, std::vector<Base*>>>;
    NodeBlocksMap _all_iports;
    // NodeBlocksMap _all_oports;
    std::vector<std::string> _nodes_with_id{"t"}; // time has an id of 0

public:
    Model(std::string name="model") : Submodel(nullptr, name) {}

    void register_iport(const Node& node, Base& block)
    {
        auto blocks = _all_iports.insert({node, {false, {}}}).first;
        if (std::find(blocks->second.second.begin(), blocks->second.second.end(), &block) == blocks->second.second.end())
            blocks->second.second.push_back(&block);
    }

    Model* get_model() override
    {
        return this;
    }

    void register_node(Node& node);
    Node get_node_by_id(Node::Id id)
    {
        return _nodes_with_id[id];
    }
};

}

#endif // __BLOCKS_HPP__
