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

#ifndef __BLOCKS_HPP__
#define __BLOCKS_HPP__

#include <algorithm>
#include <initializer_list>
#include <iterator>
#include <string>
#include <map>
#include <vector>
#include <cassert>
#include <memory>

#include "3rdparty/eigen/Eigen/Core"

using namespace Eigen;

namespace pooya
{

class Base;
class Parent;
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

class Signal
{
public:
    using Id = std::size_t;
    static constexpr Id NoId = std::string::npos;

protected:
    std::string _given_name;
    std::string _full_name;
    std::size_t _id{NoId};

    std::string _make_valid_given_name(const std::string& given_name) const;
    void _set_owner(Parent& owner);

public:
    Signal(const std::string& given_name, Parent& owner) : _given_name(_make_valid_given_name(given_name))
    {
        _set_owner(owner);
    }
    Signal(const char* given_name, Parent& owner) : _given_name(_make_valid_given_name(given_name))
    {
        _set_owner(owner);
    }
    Signal(Parent& owner) : _given_name(_make_valid_given_name(""))
    {
        _set_owner(owner);
    }
    Signal(const Signal& signal) = default;

    Signal& operator=(const Signal& rhs) = default;

    operator Id() const {return _id;}
    Id id() const {return _id;}
    const std::string& given_name() const {return _given_name;}
    const std::string& full_name() const {return _full_name;} // todo: create a shared parent with Base named NamedObject
};

inline std::ostream& operator<<(std::ostream& os, const Signal& signal)
{
    return os << "signal [" << signal.given_name() << "] = " << signal.id() << "\n";
}

struct SignalHash
{
    std::size_t operator()(const Signal& signal) const noexcept
    {
        assert(!signal.full_name().empty());
        std::size_t h = std::hash<std::string>{}(signal.full_name());
        return h ^ (signal.id() << 1);
    }
};

class Signals : public std::vector<Signal>
{
public:
    using Parent = std::vector<Signal>;
    using Parent::vector;

    Signals(const Signal& signal)
    {
        push_back(signal);
    }
};

inline std::ostream& operator<<(std::ostream& os, const Signals& signals)
{
    os << "signals:\n";
    for (const auto& signal: signals)
        os << "- " << signal;
    return os;
}

using Scalars          = std::vector<double>;
using TraverseCallback = std::function<bool(Base&, uint32_t level)>;

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

    const Value* get(Signal::Id id) const
    {
        assert(id != Signal::NoId);
        const auto& bv = _values[id];
        return bv._valid ? &bv._value : nullptr;
    }

    const Value* get(const Signal& signal) const {return get(signal.id());}

    void set(Signal::Id id, const Value& value)
    {
        assert(id != Signal::NoId);
        auto& bv = _values[id];
        assert(!bv._valid); // rewriting a valid value is not allowed
        if (!bv._valid)
        {
            bv._value = value;
            bv._valid = true;
        }
    }

    void set(const Signal& signal, const Value& value) {set(signal.id(), value);}

    void invalidate()
    {
        for (auto& v: _values) v._valid = false;
    }

    void stream(std::ostream& os) const
    {
        int k = 0;
        for (const auto& v: _values)
        {
            os << "- [" << k++ << "]: ";
            (v._valid ? os << v._value : os << "*") << "\n";
        }
        os << "\n";
    }
};

inline std::ostream& operator<<(std::ostream& os, const Values& values)
{
    values.stream(os);
    return os;
}

class StatesInfo : public std::unordered_map<Signal::Id, std::pair<Value, Signal::Id>> // state, value, derivative
{
protected:
    using Pair = std::pair<Value, Signal::Id>;
    using Parent = std::unordered_map<Signal::Id, Pair>;

public:
    using Parent::unordered_map;

    void insert_or_assign(Signal::Id state, const Value& value, Signal::Id deriv)
    {
        Parent::insert_or_assign(state, Pair(value, deriv));
    }
};

class Base
{
protected:
    Signals _iports;
    Signals _oports;
    Parent* const _parent;
    std::string _given_name;
    std::string _full_name;

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);

    Base(Parent* parent, std::string given_name, const Signals& iports=Signals(), const Signals& oports=Signals()/*, bool register_oports=true*/);

public:
    virtual ~Base() = default;

    virtual void get_states(StatesInfo& /*states*/) {}
    virtual void step(double /*t*/, const Values& /*states*/) {}
    virtual void activation_function(double /*t*/, Values& /*x*/) {}
    virtual Model& model();

    Parent* parent() {return _parent;}
    bool processed() const {return _processed;}
    const std::string& given_name() const {return _given_name;}
    const std::string& full_name() const {return _full_name;}
    const Signals& iports() const {return _iports;}
    const Signals& oports() const {return _oports;}

    virtual void _mark_unprocessed();
    virtual uint _process(double t, Values& values, bool go_deep = true);

    virtual bool traverse(TraverseCallback cb, uint32_t level, uint32_t max_level=std::numeric_limits<uint32_t>::max())
    {
        return (level > max_level) || cb(*this, level);
    }

    static std::string generate_random_name(int len = 10);
}; // class Base

// class Bus : public Base
// {
// protected:
//     bool _update_signal_ids{true};
//     std::vector<Signal> _raw_names;

// public:
//     // class BusValues : public Values
//     // {
//     // protected:
//     //     Signals _names;

//     // public:
//     //     BusValues(const Signals& names, const Values& values) :
//     //         Values(values), _names(names) {}
    
//     //     // def __repr__(self):
//     //     //     return 'BusValues(' + super().__repr__() + ')'
//     // };

//     Bus(Parent* parent, std::string given_name, const Signals& iports, const Signal& oport) :
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
    InitialValue(Parent& parent, std::string given_name, const Signal& iport, const Signal& oport) :
        Base(&parent, given_name, iport, oport) {}

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
    Const(Parent& parent, std::string given_name, const Value& value, const Signal& oport) :
        Base(&parent, given_name, Signals(), oport), _value(value) {}

    Const(Parent& parent, std::string given_name, double value, const Signal& oport) :
        Base(&parent, given_name, Signals(), oport), _value(1)
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
    Gain(Parent& parent, std::string given_name, double k, const Signal& iport, const Signal& oport) :
        Base(&parent, given_name, iport, oport), _k(k) {}

    void activation_function(double /*t*/, Values& values) override
    {
        const auto& x = *values.get(_iports[0]);
        values.set(_oports[0], _k * x);
    }
};

class Sin : public Base
{
public:
    Sin(Parent& parent, std::string given_name, const Signal& iport, const Signal& oport) :
        Base(&parent, given_name, iport, oport) {}

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
    Function(Parent& parent, std::string given_name, ActFunction act_func, const Signal& iport, const Signal& oport) :
        Base(&parent, given_name, {iport}, {oport}), _act_func(act_func) {}

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
    AddSub(Parent& parent, std::string given_name, const char* operators, const Signals& iports, const Signal& oport, double initial=0.0) :
        Base(&parent, given_name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    void activation_function(double /*t*/, Values& values) override
    {
        Value ret = Value::Constant(values.get(_iports[0])->size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& signal: _iports)
        {
            const auto &v = *values.get(signal);
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

class Add : public AddSub
{
public:
    Add(Parent& parent, std::string given_name, const Signals& iports, const Signal& oport, double initial=0.0) :
        AddSub(parent, given_name, std::string(iports.size(), '+').c_str(), iports, oport, initial) {}
};

class Subtract : public AddSub
{
public:
    Subtract(Parent& parent, std::string given_name, const Signals& iports, const Signal& oport, double initial=0.0) :
        AddSub(parent, given_name, "+-", iports, oport, initial) {}
};

class MulDiv : public Base
{
protected:
    std::string _operators;
    double      _initial;

public:
    MulDiv(Parent& parent, std::string given_name, const char* operators, const Signals& iports, const Signal& oport, double initial=1.0) :
        Base(&parent, given_name, iports, {oport}), _operators(operators), _initial(initial)
    {
        assert(std::strlen(operators) == iports.size());
    }

    void activation_function(double /*t*/, Values& values) override
    {
        Value ret = Value::Constant(values.get(_iports[0])->size(), _initial);
        const char* p = _operators.c_str();
        for (const auto& signal: _iports)
        {
            const auto &v = *values.get(signal);
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

class Multiply : public MulDiv
{
public:
    Multiply(Parent& parent, std::string given_name, const Signals& iports, const Signal& oport, double initial=1.0) :
        MulDiv(parent, given_name, std::string(iports.size(), '*').c_str(), iports, oport, initial) {}
};

class Divide : public MulDiv
{
public:
    Divide(Parent& parent, std::string given_name, const Signals& iports, const Signal& oport, double initial=1.0) :
        MulDiv(parent, given_name, "*/", iports, oport, initial) {}
};

class Integrator : public Base
{
protected:
    double _value;

public:
    Integrator(Parent& parent, std::string given_name, const Signal& iport, const Signal& oport, double ic=0.0) :
        Base(&parent, given_name, Signals({iport}), Signals({oport})), _value(ic) {}

    void get_states(StatesInfo& states) override
    {
        states.insert_or_assign(_oports.front(), Value(_value), _iports.front());
    }

    void step(double /*t*/, const Values& values) override
    {
        assert(values.get(_oports.front()));
        _value = (*values.get(_oports.front()))[0];
    }

    uint _process(double t, Values& values, bool go_deep = true) override;
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
    Delay(Parent& parent, std::string given_name, const Signals& iports, const Signal& oport, double lifespan=10.0) :
        Base(&parent, given_name, iports, oport), _lifespan(lifespan) {}

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
    Memory(Parent& parent, std::string given_name, const Signal& iport, const Signal& oport, const Value& ic=Value::Zero(1)) :
        Base(&parent, given_name, Signals({iport}), Signals({oport})), _value(ic) {}

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

    uint _process(double t, Values& values, bool go_deep = true) override;
};

class Derivative : public Base
{
protected:
    bool _first_step{true};
    double _t;
    Value  _x;
    Value  _y;

public:
    Derivative(Parent& parent, std::string given_name, const Signal& iport, const Signal& oport, const Value& y0=Value::Zero(1)) :
        Base(&parent, given_name, iport, oport), _y(y0) {}

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

class Parent : public Base
{
protected:
    std::vector<std::unique_ptr<Base>> _components;

    Parent(Parent* parent, std::string given_name, const Signals& iports=Signals(), const Signals& oports=Signals()) :
        Base(parent, given_name, iports, oports/*, false*/) {}

public:
    bool add_component(Base& component)
    {
        if (component.parent() != this) return false;
        for (auto it = _components.begin(); it != _components.end(); it++)
            if (it->get() == &component) return false;
        _components.emplace_back(&component);
        return true;
    }

    void get_states(StatesInfo& states) override
    {
        for (auto it = _components.begin(); it != _components.end(); it++)
            (*it)->get_states(states);
    }

    void step(double t, const Values& values) override
    {
        for (auto& component: _components)
            component->step(t, values);
    }

    Signal signal(const std::string& given_name="")
    {
        return Signal(given_name, *this);
    }

    std::string make_signal_name(const std::string& given_name);
    Signal parameter(const std::string& given_name);
    void _mark_unprocessed() override;
    uint _process(double t, Values& values, bool go_deep = true) override;
    bool traverse(TraverseCallback cb, uint32_t level, uint32_t max_level=std::numeric_limits<uint32_t>::max()) override;
}; // class Parent

class Submodel : public Parent
{
public:
    Submodel(Parent& parent, std::string given_name, const Signals& iports=Signals(), const Signals& oports=Signals()) :
        Parent(&parent, given_name, iports, oports/*, false*/) {}

}; // class Submodel

class Model : public Parent
{
protected:
    std::vector<std::string> _registered_signals;

public:
    Model(std::string name="model");

    Model& model() override {return *this;}
    std::size_t num_signals() const {return _registered_signals.size();}

    const std::string& get_signal_by_id(Signal::Id id) const
    {
        return _registered_signals[id];
    }

    Signal::Id find_signal(const std::string& name, bool exact_match = false) const;
    Signal::Id find_or_register_signal(const std::string& name);
};

inline Signal Parent::parameter(const std::string& given_name)
{
    return Signal(given_name, model());
}

}

#endif // __BLOCKS_HPP__
