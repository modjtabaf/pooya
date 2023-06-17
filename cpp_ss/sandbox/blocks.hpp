
#ifndef __BLOCKS_HPP__
#define __BLOCKS_HPP__

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

    Value(double v) : ArrayXd(1)
    {
        (*this)[0] = v;
    }
};

class StateData
{
public:
    StateData(const std::string& deriv, const Value& value) :
        _deriv(deriv), _value(value) {}
    StateData(const std::string& deriv, double value) :
        _deriv(deriv), _value(1)
    {
        _value[0] = value;
    }

    std::string _deriv;
    Value       _value;
};

class Node : public std::string
{
public:
    using Parent = std::string;

    Node(const std::string& str) : std::string(str) {}
    Node(const char* str, bool name_locked=true) : std::string(str) {}
    Node(int n) : std::string(std::to_string(n)) {}
    Node() : std::string("-") {}

    using Parent::operator=;
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

using Scalars          = std::vector<double>;
using Values           = std::vector<Value>;
using NodeValues       = std::map<Node, Value>;
using States           = std::map<std::string, StateData>;
using TraverseCallback = std::function<bool(const Base&)>;
using ActFunction      = std::function<Value(double, const Value&)>;

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
    Base(const char* name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes(), bool register_oports=true);

    virtual void get_states(States& states) {}
    virtual void step(double t, const NodeValues& states) {}
    virtual Values activation_function(double t, const Values& x)
    {
        assert(false);
        return Values();
    }

    // def get_ports(self):
    //     return self._iports, self._oports

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

// class Bus(Base):
//     class BusValues(list):
//         def __init__(self, names, *args):
//             super().__init__(args)
//             self._names = names
        
//         def __repr__(self):
//             return 'BusValues(' + super().__repr__() + ')'

//     def __init__(self, name, iports='-', oport='-'):
//         super().__init__(name, iports, oport)
        
//         if not isinstance(iports, (list, tuple)):
//             iports = [iports]
//         self._raw_names = [n for n in iports]

//     def activation_function(self, t, x):
//         ret = [Bus.BusValues(self._raw_names, *x)]
//         return ret

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

// class InitialValue(Base):
//     def __init__(self, name, iport='-', oport='-'):
//         super().__init__(name, iport, oport)
//         self._value = None

//     def activation_function(self, t, x):
//         if self._value is None:
//             self._value = x[0]
//             self._iports = []
//         return [self._value]

class Const : public Base
{
protected:
    Value _value;

public:
    Const(const char* name, const Value& value, const Nodes& oport=Nodes({Node()})) :
        Base(name, Nodes(), oport), _value(value) {}

    Const(const char* name, double value, const Nodes& oport={Node()}) :
        Base(name, Nodes(), oport), _value(1)
    {
         _value << value;
    }

    Values activation_function(double t, const Values& x) override
    {
        return Values{{_value}};
    }
};

class Gain : public Base
{
protected:
    double _k;

public:
    Gain(const char* name, double k, const Nodes& iport=Nodes({Node()}), const Nodes& oport=Nodes({Node()})) :
        Base(name, iport, oport), _k(k) {}

    Values activation_function(double t, const Values& x) override
    {
        return {_k * x[0]};
    }
};

class Sin : public Base
{
public:
    Sin(const char* name, const Nodes& iports=Nodes({Node()}), const Nodes& oports=Nodes({Node()})) :
        Base(name, iports, oports) {}

    Values activation_function(double t, const Values& x) override
    {
        return {x[0].sin()};
    }
};

class Function : public Base
{
protected:
    ActFunction _act_func;

public:
    Function(const char* name, ActFunction act_func, const Node& iport=Node(), const Node& oport=Node()) :
        Base(name, {iport}, {oport}), _act_func(act_func) {}

    Values activation_function(double t, const Values& x) override
    {
        return {_act_func(t, x[0])};
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
    AddSub(const char* name, const char* operators, const Nodes& iports, const Node& oport=Node(), double initial=0.0) :
        Base(name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    Values activation_function(double t, const Values& x) override
    {
        Value ret = Value::Constant(x[0].size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& v: x)
        {
            if (*p == '+')
                ret += v;
            else if (*p == '-')
                ret -= v;
            else
                 assert(false);
            p++;
        }
        return {ret};
    }
};

class MulDiv : public Base
{
protected:
    std::string _operators;
    double      _initial;

public:
    MulDiv(const char* name, const char* operators, const Nodes& iports, const Node& oport=Node(), double initial=1.0) :
        Base(name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    Values activation_function(double t, const Values& x) override
    {
        Value ret = Value::Constant(x[0].size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& v: x)
        {
            if (*p == '*')
                ret *= v;
            else if (*p == '/')
                ret /= v;
            else
                 assert(false);
            p++;
        }
        return {ret};
    }
};

class Integrator : public Base
{
protected:
    double _value;

public:
    Integrator(const char* name, const Node& iport=Node(), const Node& oport=Node(), double ic=0.0) :
        Base(name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void get_states(States& states) override
    {
        states.insert_or_assign(_oports.front(), StateData(_iports.front(), _value));
    }

    void step(double t, const NodeValues& states) override
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
    Delay(const char* name, const Nodes& iports, const Nodes& oport=Nodes({Node()}), double lifespan=10.0) :
        Base(name, iports, oport), _lifespan(lifespan) {}

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

    Values activation_function(double t, const Values& x) override
    {
        if (_t.empty())
            return {x[2][0]};
    
        const double delay = x[1][0];
        t -= delay;
        if (t <= _t.front())
        {
            return {x[2]};
        }
        else if (t >= _t.back())
        {
            return {_x.back()};
        }

        int k = 0;
        for (const auto& v: _t)
        {
            if (v >= t)
                break;
            k++;
        }

        return {(_x[k][0] - _x[k - 1][0])*(t - _t[k - 1])/(_t[k] - _t[k - 1]) + _x[k - 1][0]};
    }
};

class Memory : public Base
{
protected:
    Value _value;

public:
    Memory(const char* name, const Node& iport=Node(), const Node& oport=Node(), const Value& ic=Value::Zero(1)) :
        Base(name, Nodes({iport}), Nodes({oport})), _value(ic) {}

    void step(double t, const NodeValues& states) override
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

// class Derivative(Base):
//     def __init__(self, name, iport='-', oport='-', y0=0):
//         super().__init__(name, iports=iport, oports=oport)
//         self._t = None
//         self._x = None
//         self._y = y0

//     def step(self, t, states):
//         self._t = t
//         self._x = states[self._iports[0]]
//         self._y = states[self._oports[0]]

//     def activation_function(self, t, x):
//         if self._t is None:
//             self._t = t
//             self._x = x[0]
//             return [self._y]
//         elif self._t == t:
//             return [self._y]
//         else:
//             return [(x[0] - self._x)/(t - self._t)]

class Submodel : public Base
{
protected:
    static std::vector<Submodel*> _current_submodels;

public:
    static Submodel* current()
    {
        return Submodel::_current_submodels.empty() ? nullptr : Submodel::_current_submodels.back();
    }

protected:
    std::vector<Base*> _components;
    std::string _auto_signal_name;

public:
    Submodel(const char* name, const Nodes& iports=Nodes(), const Nodes& oports=Nodes()) :
        Base(name, iports, oports, false) {}

    void enter() {_current_submodels.push_back(this);}
    void exit()
    {
        assert(_current_submodels.back() == this);
        _current_submodels.pop_back();
    }

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
