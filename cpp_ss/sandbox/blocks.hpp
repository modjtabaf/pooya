
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

class Value : public ArrayXd
{
public:
    using ArrayXd::ArrayXd;

    Value(double v=0) : ArrayXd(1)
    {
        (*this)[0] = v;
    }
};

class Node
{
public:
    using Id = std::size_t;
    static const Id NoId = std::string::npos;

protected:
    std::string _given_name;
    std::string _name;
    std::size_t _id{NoId};

    std::string _make_valid_given_name(const std::string& given_name) const;

public:
    Node(const std::string& given_name) : _given_name(_make_valid_given_name(given_name)) {}
    Node(const char* given_name) : _given_name(_make_valid_given_name(given_name)) {}
    Node(int n) : _given_name(_make_valid_given_name(std::to_string(n))) {}
    Node() : _given_name(_make_valid_given_name("-")) {}
    Node(const Node& node) = default;

    Node& operator=(const Node& rhs) = default;

    operator Id() const {return _id;}
    Id id() const {return _id;}
    const std::string& given_name() const {return _given_name;}
    const std::string& name() const {return _name;} // todo: create a shared parent with Base named NamedObject

    bool set_owner(Submodel& owner);
};

inline std::ostream& operator<<(std::ostream& os, const Node& node)
{
    return os << "node [" << node.given_name() << "] = " << node.id() << "\n";
}

struct NodeHash
{
    std::size_t operator()(const Node& node) const noexcept
    {
        assert(!node.name().empty());
        std::size_t h = std::hash<std::string>{}(node.name());
        return h ^ (node.id() << 1);
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

inline std::ostream& operator<<(std::ostream& os, const Nodes& nodes)
{
    os << "nodes:\n";
    for (const auto& node: nodes)
        os << "- " << node;
    return os;
}

using Scalars          = std::vector<double>;
using TraverseCallback = std::function<bool(const Base&, uint32_t level)>;

class NodeValues : public std::unordered_map<Node, Value, NodeHash>
{
protected:
    using Parent = std::unordered_map<Node, Value, NodeHash>;

public:
    NodeValues() = default;
    NodeValues(const std::initializer_list<std::pair<Node, Value>>& list, Submodel& owner);
    NodeValues(const std::initializer_list<std::pair<Node, Value>>& list)
    {
        for (const auto& v: list)
            insert_or_assign(v.first, v.second);
    }

    void join(const NodeValues& other)
    {
        for(const auto& value: other)
            insert_or_assign(value.first, value.second);
    }
};

class Values
{
protected:
    struct BoolValue
    {
        bool _valid{false};
        Value _value;
    };
    std::vector<BoolValue> _values;

public:
    Values(std::size_t n)
    {
        _values.reserve(n);
        for (std::size_t k=0; k < n; k++)
            _values.emplace_back(BoolValue());
    }

    std::size_t size() const {return _values.size();}

    const Value* get(Node::Id id) const
    {
        assert(id != Node::NoId);
        const auto& bv = _values[id];
        return bv._valid ? &bv._value : nullptr;
    }

    void set(Node::Id id, const Value& value)
    {
        assert(id != Node::NoId);
        auto& bv = _values[id];
        bv._value = value;
        bv._valid = true;
    }

    void invalidate()
    {
        for (auto& v: _values) v._valid = false;
    }

    void stream(std::ostream& os) const
    {
        int k = 0;
        for (const auto& v: _values)
        {
            if (v._valid)
                os << "- " << k << ": " << v._value << "\n";
            k++;
        }
        os << "\n";
    }
};

inline std::ostream& operator<<(std::ostream& os, const Values& values)
{
    values.stream(os);
    return os;
}

class StatesInfo : public std::unordered_map<Node::Id, std::pair<Value, Node::Id>> // state, value, derivative
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

class Base
{
protected:
    Nodes _iports;
    Nodes _oports;
    Submodel* const _parent;
    std::string _given_name;
    std::string _name;

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);

public:
    Base(Submodel* parent, std::string given_name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes()/*, bool register_oports=true*/);

    virtual void get_states(StatesInfo& /*states*/) {}
    virtual void step(double /*t*/, const Values& /*states*/) {}
    virtual void activation_function(double /*t*/, Values& /*x*/) {}
    virtual Model* get_model();

    bool processed() const {return _processed;}
    const std::string& given_name() const {return _given_name;}
    const std::string& name() const {return _name;}
    const Nodes& iports() const {return _iports;}
    const Nodes& oports() const {return _oports;}

    virtual uint _process(double t, Values& values, bool reset);

    virtual bool traverse(TraverseCallback cb, uint32_t level, decltype(level) max_level=std::numeric_limits<decltype(level)>::max())
    {
        return (level > max_level) || cb(*this, level);
    }

    static std::string generate_random_name(int len = 10);
}; // class Base

// class Bus : public Base
// {
// protected:
//     bool _update_node_ids{true};
//     std::vector<Node> _raw_names;

// public:
//     // class BusValues : public Values
//     // {
//     // protected:
//     //     Nodes _names;

//     // public:
//     //     BusValues(const Nodes& names, const Values& values) :
//     //         Values(values), _names(names) {}
    
//     //     // def __repr__(self):
//     //     //     return 'BusValues(' + super().__repr__() + ')'
//     // };

//     Bus(Submodel* parent, std::string given_name, const Nodes& iports=Node(), const Node& oport=Node()) :
//         Base(parent, given_name, iports, oport)
//     {
//         _raw_names.reserve(iports.size());
//         for (const auto& p: iports)
//             _raw_names.push_back(p);
//     }

//     void activation_function(double /*t*/, Values& /*x*/) override;
// };

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

    void activation_function(double /*t*/, Values& values) override
    {
        if (_value.size() == 0)
        {
            _value = *values.get(_iports[0]);
            _iports.clear();
        }
        values.set(_oports[0], _value);
    }
};

class Const : public Base
{
protected:
    Value _value;

public:
    Const(Submodel* parent, std::string given_name, const Value& value, const Node& oport=Node()) :
        Base(parent, given_name, Nodes(), oport), _value(value) {}

    Const(Submodel* parent, std::string given_name, double value, const Node& oport=Node()) :
        Base(parent, given_name, Nodes(), oport), _value(1)
    {
         _value << value;
    }

    void activation_function(double /*t*/, Values& values) override
    {
        values.set(_oports[0], _value);
    }
};

class Gain : public Base
{
protected:
    double _k;

public:
    Gain(Submodel* parent, std::string given_name, double k, const Node& iport=Node(), const Node& oport=Node()) :
        Base(parent, given_name, iport, oport), _k(k) {}

    void activation_function(double /*t*/, Values& values) override
    {
        const auto& x = *values.get(_iports[0]);
        values.set(_oports[0], _k * x);
    }
};

class Sin : public Base
{
public:
    Sin(Submodel* parent, std::string given_name, const Node& iport=Node(), const Node& oport=Node()) :
        Base(parent, given_name, iport, oport) {}

    void activation_function(double /*t*/, Values& values) override
    {
        values.set(_oports[0], values.get(_iports[0])->sin());
    }
};

class Function : public Base
{
public:
    using ActFunction = std::function<Value(double, const Value&)>;

protected:
    ActFunction _act_func;

public:
    Function(Submodel* parent, std::string given_name, ActFunction act_func, const Node& iport=Node(), const Node& oport=Node()) :
        Base(parent, given_name, {iport}, {oport}), _act_func(act_func) {}

    void activation_function(double t, Values& values) override
    {
        values.set(_oports[0], _act_func(t, *values.get(_iports[0])));
    }
};

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

    void activation_function(double /*t*/, Values& values) override
    {
        Value ret = Value::Constant(values.get(_iports[0])->size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& node: _iports)
        {
            const auto &v = *values.get(node);
            if (*p == '+')
                ret += v;
            else if (*p == '-')
                ret -= v;
            else
                 assert(false);
            p++;
        }
        values.set(_oports[0], ret);
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

    void activation_function(double /*t*/, Values& values) override
    {
        Value ret = Value::Constant(values.get(_iports[0])->size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& node: _iports)
        {
            const auto &v = *values.get(node);
            if (*p == '*')
                ret *= v;
            else if (*p == '/')
                ret /= v;
            else
                 assert(false);
            p++;
        }
        values.set(_oports[0], ret);
    }
};

class Integrator : public Base
{
protected:
    double _value;

public:
    Integrator(Submodel* parent, std::string given_name, const Node& iport=Node(), const Node& oport=Node(), double ic=0.0) :
        Base(parent, given_name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void get_states(StatesInfo& states) override
    {
        states.insert_or_assign(_oports.front(), Value(_value), _iports.front());
    }

    void step(double /*t*/, const Values& values) override
    {
        assert(values.get(_oports.front()));
        _value = (*values.get(_oports.front()))[0];
    }

    uint _process(double t, Values& values, bool reset) override;
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
    std::vector<Value> _x;

public:
    Delay(Submodel* parent, std::string given_name, const Nodes& iports, const Node& oport=Node(), double lifespan=10.0) :
        Base(parent, given_name, iports, oport), _lifespan(lifespan) {}

    void step(double t, const Values& values) override
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
        assert(values.get(_iports.front()));
        _x.push_back(*values.get(_iports.front()));
    }

    void activation_function(double t, Values& values) override
    {
        if (_t.empty())
        {
            assert(values.get(_iports[2]));
            values.set(_oports.front(), {(*values.get(_iports[2]))[0]});
            return;
        }
    
        assert(values.get(_iports[1]));
        const double delay = (*values.get(_iports[1]))[0];
        t -= delay;
        if (t <= _t.front())
        {
            assert(values.get(_iports[2]));
            values.set(_oports.front(), *values.get(_iports[2]));
        }
        else if (t >= _t.back())
        {
            values.set(_oports.front(), _x.back());
        }
        else
        {
            int k = 0;
            for (const auto& v: _t)
            {
                if (v >= t)
                    break;
                k++;
            }

            values.set(_oports.front(), (_x[k][0] - _x[k - 1][0])*(t - _t[k - 1])/(_t[k] - _t[k - 1]) + _x[k - 1][0]);
        }
    }
};

class Memory : public Base
{
protected:
    Value _value;

public:
    Memory(Submodel* parent, std::string given_name, const Node& iport=Node(), const Node& oport=Node(), const Value& ic=Value::Zero(1)) :
        Base(parent, given_name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void step(double /*t*/, const Values& values) override
    {
        assert(values.get(_iports.front()));
        _value = *values.get(_iports.front());
    }

    // # Memory can be implemented either by defining the following activation function
    // #   (which is more straightforward) or through overloading the _process method
    // #   which is more efficient since it deosn't rely on the input signal being known.
    // #   Both approaches are supposed to lead to the exact same results.

    // # def activation_function(self, t, x):
    // #     return [self._value]

    uint _process(double t, Values& values, bool reset) override;
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

    void step(double t, const Values& states) override
    {
        _t = t;
        _x = *states.get(_iports[0]);
        _y = *states.get(_oports[0]);
        _first_step = false;
    }

    void activation_function(double t, Values& values) override
    {
        if (_first_step)
        {
            _t = t;
            _x = *values.get(_iports[0]);
            values.set(_oports[0], _y);
        }
        else if (_t == t)
            values.set(_oports[0], _y);
        else
            values.set(_oports[0], ((*values.get(_iports[0]) - _x)/(t - _t)));
    }
};

class Submodel : public Base
{
protected:
    std::vector<Base*> _components;

public:
    Submodel(Submodel* parent, std::string given_name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes()) :
        Base(parent, given_name, iports, oports/*, false*/) {}

    void add_component(Base& component)
    {
        _components.push_back(&component);
    }

    void get_states(StatesInfo& states) override
    {
        for (auto* component: _components)
            component->get_states(states);
    }

    void step(double t, const Values& values) override
    {
        for (auto& component: _components)
            component->step(t, values);
    }

    std::string make_node_name(const std::string& given_name);
    Node create_node(const std::string& given_name);
    uint _process(double t, Values& values, bool reset) override;
    bool traverse(TraverseCallback cb, uint32_t level, decltype(level) max_level=std::numeric_limits<decltype(level)>::max()) override;

}; // class Submodel

class Model final : public Submodel
{
protected:
    std::vector<std::string> _registered_nodes;

public:
    Model(std::string name="model");

    Model* get_model() override {return this;}
    std::size_t num_nodes() const {return _registered_nodes.size();}

    const std::string& get_node_by_id(Node::Id id) const
    {
        return _registered_nodes[id];
    }
    Node::Id get_or_register_node(const std::string& name);
};

}

#endif // __BLOCKS_HPP__
