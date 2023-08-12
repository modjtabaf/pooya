
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
protected:
    bool _is_locked{false};

public:
    using Parent = std::string;

    Node(const std::string& str) : std::string(str) {}
    Node(const char* str) : std::string(str) {}
    Node(int n) : std::string(std::to_string(n)) {}
    Node() : std::string("-") {}
    Node(const Node& node) = default;

    void lock() {_is_locked = true;}
    bool is_locked() const {return _is_locked;}

    Node& operator=(const Node& rhs) = default;
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

class Base;
class Submodel;
class NodeValues;

using Scalars          = std::vector<double>;
using Values           = std::vector<Value>;
using TraverseCallback = std::function<bool(const Base&)>;
using ActFunction      = std::function<Value(double, const Value&)>;

std::ostream& operator<<(std::ostream&, const class NodeValues&);

class NodeValues : public std::pair<Nodes, Values>
{
protected:
    using Parent = std::pair<Nodes, Values>;

public:
    using Parent::pair;
    NodeValues(const std::initializer_list<std::pair<Node, Value>>& list)
    {
        first.reserve(list.size());
        second.reserve(list.size());
        for (const auto& v: list)
        {
            first.push_back(v.first);
            second.push_back(v.second);
        }
    }

    Nodes::const_iterator find(const Node& node) const
    {
        assert(first.size() == second.size());
        return std::find(first.cbegin(), first.cend(), node);
    }

    const Value& at(const Node& node) const
    {
        assert(first.size() == second.size());
        auto k = std::distance(first.begin(), find(node));
        return second.at(k);
    }

    void insert_or_assign(const Node& node, const Value& value)
    {
        assert(first.size() == second.size());
        auto it = find(node);
        if (it == first.cend())
        {
            first.push_back(node);
            second.push_back(value);
        }
        else
        {
            auto k = std::distance(first.cbegin(), it);
            second.at(k) = value;
        }
    }

    void join(const NodeValues& other)
    {
        assert(first.size() == second.size());
        auto node = other.first.begin();
        for(const auto& value: other.second)
            insert_or_assign(*(node++), value);
    }
};

class States : public std::tuple<Nodes, Values, Nodes> // state, value, derivative
{
protected:
    using Parent = std::tuple<Nodes, Values, Nodes>;

public:
    using Parent::tuple;

    Nodes::const_iterator find(const Node& state) const
    {
        const auto& states = std::get<0>(*this);
        assert(states.size() == std::get<1>(*this).size());
        assert(states.size() == std::get<2>(*this).size());
        return std::find(states.cbegin(), states.cend(), state);
    }

    std::pair<Value, Node> at(const Node& state) const
    {
        const auto& states = std::get<0>(*this);
        assert(states.size() == std::get<1>(*this).size());
        assert(states.size() == std::get<2>(*this).size());
        auto k = std::distance(states.begin(), find(state));
        return {std::get<1>(*this).at(k), std::get<2>(*this).at(k)};
    }

    void insert_or_assign(const Node& state, const Value& value, const Node& deriv)
    {
        auto& states = std::get<0>(*this);
        assert(states.size() == std::get<1>(*this).size());
        assert(states.size() == std::get<2>(*this).size());
        auto it = find(state);
        if (it == states.cend())
        {
            states.push_back(state);
            std::get<1>(*this).push_back(value);
            std::get<2>(*this).push_back(deriv);
        }
        else
        {
            auto k = std::distance(states.cbegin(), it);
            std::get<1>(*this).at(k) = value;
            std::get<2>(*this).at(k) = deriv;
        }
    }
};

inline std::ostream& operator<<(std::ostream& os, const NodeValues& nv)
{
    auto node = nv.first.begin();
    for (auto& value: nv.second)
    {
        os << "  - " << *node << ": " << value << "\n";
        node++;
    }
    return os << "\n";
}

class Base
{
protected:
    static Nodes _all_iports;
    static Nodes _all_oports;

    Nodes _iports;
    Nodes _oports;
    NodeValues _known_values;

    std::string _name;
    bool _processed{false};

public:
    Base(Submodel* parent, const char* name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes(), bool register_oports=true);

    virtual void get_states(States& /*states*/) {}
    virtual void step(double /*t*/, const NodeValues& /*states*/) {}
    virtual NodeValues activation_function(double /*t*/, const NodeValues& /*x*/)
    {
        assert(false);
        return NodeValues();
    }

    bool is_processed() const {return _processed;}
    const std::string& name() const {return _name;}
    const Nodes& iports() const {return _iports;}
    const Nodes& oports() const {return _oports;}

    virtual uint _process(double t, NodeValues& x, bool reset);

    // def __repr__(self):
    //     return str(type(self)) + ":" + self._name + ", iports:" + str(self._iports) + ", oports:" + str(self._oports)

    virtual bool traverse(TraverseCallback cb)
    {
        return cb(*this);
    }
}; // class Base

// class Value(Base):
//     def activation_function(self, t, x):
//         return [x[0]]

class Bus : public Base
{
protected:
    std::vector<std::string> _raw_names;

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

    Bus(Submodel* parent, const char* name, const Nodes& iports=Node(), const Node& oport=Node()) :
        Base(parent, name, iports, oport)
    {
        _raw_names.reserve(iports.size());
        for (const auto& p: iports)
            _raw_names.push_back(p);
    }

    NodeValues activation_function(double /*t*/, const NodeValues& x) override
    {
        assert(x.first.size() == _raw_names.size());
        NodeValues ret;
        ret.first.reserve(_raw_names.size());
        ret.second.reserve(_raw_names.size());
        auto p = x.second.begin();
        for (const auto& name: _raw_names)
        {
            ret.insert_or_assign(name, *(p++));
        }
        return ret;
    }
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
    InitialValue(Submodel* parent, const char* name, const Node& iport=Node(), const Node& oport=Node()) :
        Base(parent, name, iport, oport) {}

    NodeValues activation_function(double /*t*/, const NodeValues& x) override
    {
        if (_value.size() == 0)
        {
            _value = x.second[0];
            _iports.clear();
        }
        return NodeValues(_oports, {_value});
    }
};

class Const : public Base
{
protected:
    Value _value;

public:
    Const(Submodel* parent, const char* name, const Value& value, const Nodes& oport=Nodes({Node()})) :
        Base(parent, name, Nodes(), oport), _value(value) {}

    Const(Submodel* parent, const char* name, double value, const Nodes& oport={Node()}) :
        Base(parent, name, Nodes(), oport), _value(1)
    {
         _value << value;
    }

    NodeValues activation_function(double /*t*/, const NodeValues& /*x*/) override
    {
        return NodeValues(_oports, {_value});
    }
};

class Gain : public Base
{
protected:
    double _k;

public:
    Gain(Submodel* parent, const char* name, double k, const Nodes& iport=Nodes({Node()}), const Nodes& oport=Nodes({Node()})) :
        Base(parent, name, iport, oport), _k(k) {}

    NodeValues activation_function(double /*t*/, const NodeValues& x) override
    {
        return NodeValues(_oports, {_k * x.second[0]});
    }
};

class Sin : public Base
{
public:
    Sin(Submodel* parent, const char* name, const Nodes& iports=Nodes({Node()}), const Nodes& oports=Nodes({Node()})) :
        Base(parent, name, iports, oports) {}

    NodeValues activation_function(double /*t*/, const NodeValues& x) override
    {
        return NodeValues(_oports, {x.second[0].sin()});
    }
};

class Function : public Base
{
protected:
    ActFunction _act_func;

public:
    Function(Submodel* parent, const char* name, ActFunction act_func, const Node& iport=Node(), const Node& oport=Node()) :
        Base(parent, name, {iport}, {oport}), _act_func(act_func) {}

    NodeValues activation_function(double t, const NodeValues& x) override
    {
        return NodeValues(_oports, {_act_func(t, x.second[0])});
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
    AddSub(Submodel* parent, const char* name, const char* operators, const Nodes& iports, const Node& oport=Node(), double initial=0.0) :
        Base(parent, name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    NodeValues activation_function(double /*t*/, const NodeValues& x) override
    {
        Value ret = Value::Constant(x.second[0].size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& v: x.second)
        {
            if (*p == '+')
                ret += v;
            else if (*p == '-')
                ret -= v;
            else
                 assert(false);
            p++;
        }
        return NodeValues(_oports, {ret});
    }
};

class MulDiv : public Base
{
protected:
    std::string _operators;
    double      _initial;

public:
    MulDiv(Submodel* parent, const char* name, const char* operators, const Nodes& iports, const Node& oport=Node(), double initial=1.0) :
        Base(parent, name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    NodeValues activation_function(double /*t*/, const NodeValues& x) override
    {
        Value ret = Value::Constant(x.second[0].size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& v: x.second)
        {
            if (*p == '*')
                ret *= v;
            else if (*p == '/')
                ret /= v;
            else
                 assert(false);
            p++;
        }
        return NodeValues(_oports, {ret});
    }
};

class Integrator : public Base
{
protected:
    double _value;

public:
    Integrator(Submodel* parent, const char* name, const Node& iport=Node(), const Node& oport=Node(), double ic=0.0) :
        Base(parent, name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void get_states(States& states) override
    {
        states.insert_or_assign(_oports.front(), _value, _iports.front());
    }

    void step(double /*t*/, const NodeValues& states) override
    {
        _value = states.at(_oports.front())[0];
    }

    uint _process(double t, NodeValues& x, bool reset) override;
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
    Delay(Submodel* parent, const char* name, const Nodes& iports, const Nodes& oport=Nodes({Node()}), double lifespan=10.0) :
        Base(parent, name, iports, oport), _lifespan(lifespan) {}

    void step(double t, const NodeValues& states) override
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
        _x.push_back(states.at(_iports.front()));
    }

    NodeValues activation_function(double t, const NodeValues& x) override
    {
        if (_t.empty())
            return NodeValues(_oports, {x.second[2][0]});
    
        const double delay = x.second[1][0];
        t -= delay;
        if (t <= _t.front())
        {
            return NodeValues(_oports, {x.second[2]});
        }
        else if (t >= _t.back())
        {
            return NodeValues(_oports, {_x.back()});
        }

        int k = 0;
        for (const auto& v: _t)
        {
            if (v >= t)
                break;
            k++;
        }

        return NodeValues(_oports, {(_x[k][0] - _x[k - 1][0])*(t - _t[k - 1])/(_t[k] - _t[k - 1]) + _x[k - 1][0]});
    }
};

class Memory : public Base
{
protected:
    Value _value;

public:
    Memory(Submodel* parent, const char* name, const Node& iport=Node(), const Node& oport=Node(), const Value& ic=Value::Zero(1)) :
        Base(parent, name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void step(double /*t*/, const NodeValues& states) override
    {
        _value = states.at(_iports.front());
    }

    // # Memory can be implemented either by defining the following activation function
    // #   (which is more straightforward) or through overloading the _process method
    // #   which is more efficient since it deosn't rely on the input signal being known.
    // #   Both approaches are supposed to lead to the exact same results.

    // # def activation_function(self, t, x):
    // #     return [self._value]

    uint _process(double t, NodeValues& x, bool reset) override;
};

class Derivative : public Base
{
protected:
    bool _first_step{true};
    double _t;
    Value  _x;
    Value  _y;

public:
    Derivative(Submodel* parent, const char* name, const Node& iport=Node(), const Node& oport=Node(), const Value& y0=Value::Zero(1)) :
        Base(parent, name, iport, oport), _y(y0) {}

    void step(double t, const NodeValues& states) override
    {
        _t = t;
        _x = states.at(_iports.front());
        _y = states.at(_oports.front());
        _first_step = false;
    }

    NodeValues activation_function(double t, const NodeValues& x) override
    {
        if (_first_step)
        {
            _t = t;
            _x = x.second[0];
            return NodeValues(_oports, {_y});
        }
        else if (_t == t)
            return NodeValues(_oports, {_y});

        return NodeValues(_oports, {((x.second[0] - _x)/(t - _t))});
    }
};

class Submodel : public Base
{
protected:
    std::vector<Base*> _components;
    std::string _auto_node_name;

public:
    Submodel(Submodel* parent, const char* name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes()) :
        Base(parent, name, iports, oports, false) {}

    void add_component(Base& component)
    {
        _components.push_back(&component);
    }

    void get_states(States& states) override
    {
        for (auto* component: _components)
            component->get_states(states);
    }

    void step(double t, const NodeValues& states) override
    {
        for (auto& component: _components)
            component->step(t, states);
    }

    Node get_node_name(const Node& node, bool makenew);
    uint _process(double t, NodeValues& x, bool reset) override;
    bool traverse(TraverseCallback cb) override;

}; // class Submodel

}

#endif // __BLOCKS_HPP__
