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
    Signal() = default;
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

    const Value& operator[](Signal::Id id) const
    {
        const Value* ret = get(id);
        assert(ret); // TODO: throw an exception if null
        return *ret;
    }

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

    void add(Signal::Id state, const Value& value, Signal::Id deriv)
    {
        if (Parent::find(state) == end())
            Parent::insert({state, Pair(value, deriv)});
        else
            assert(false);
    }
};

class Base
{
    friend class Parent;

protected:
    Signals _iports;
    Signals _oports;
    Signals _dependencies;
    Parent* _parent{nullptr};
    std::string _given_name;
    std::string _full_name;

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);
    bool _add_dependecny(const Signal& signal);

    Base(std::string given_name) : _given_name(given_name) {}

    virtual bool init(Parent& parent, const Signals& iports={}, const Signals& oports={});

public:
    virtual ~Base() = default;

    virtual void get_states(StatesInfo& /*states*/) {}
    virtual void step(double /*t*/, const Values& /*states*/) {}
    virtual void activation_function(double /*t*/, Values& /*x*/) {}
    virtual Model* model();

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

class InitialValue : public Base
{
protected:
    Value _value;

public:
    InitialValue(std::string given_name) : Base(given_name) {}

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
    Const(std::string given_name, const Value& value) : Base(given_name), _value(value) {}

    Const(std::string given_name, double value) : Base(given_name), _value(1)
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
    Gain(std::string given_name, double k) : Base(given_name), _k(k) {}

    void activation_function(double /*t*/, Values& values) override
    {
        const auto& x = *values.get(_iports[0]);
        values.set(_oports[0], _k * x);
    }
};

class Sin : public Base
{
public:
    Sin(std::string given_name) : Base(given_name) {}

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
    Function(std::string given_name, ActFunction act_func) :
        Base(given_name), _act_func(act_func) {}

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
    AddSub(std::string given_name, const char* operators, double initial=0.0) :
        Base(given_name), _operators(operators), _initial(initial)
    {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        assert(_operators.size() == iports.size());
        return (_operators.size() == iports.size()) &&
            Base::init(parent, iports, oports);
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
    Add(std::string given_name, double initial=0.0) :
        AddSub(given_name, "", initial) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        _operators = std::string(iports.size(), '+');
        return AddSub::init(parent, iports, oports);
    }
};

class Subtract : public AddSub
{
public:
    Subtract(std::string given_name, double initial=0.0) :
        AddSub(given_name, "+-", initial) {}
};

class MulDiv : public Base
{
protected:
    std::string _operators;
    double      _initial;

public:
    MulDiv(std::string given_name, const char* operators, double initial=1.0) :
        Base(given_name), _operators(operators), _initial(initial) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        assert(_operators.size() == iports.size());
        return (_operators.size() == iports.size()) &&
            Base::init(parent, iports, oports);
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
    Multiply(std::string given_name, double initial=1.0) :
        MulDiv(given_name, "", initial) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        _operators = std::string(iports.size(), '*');
        return MulDiv::init(parent, iports, oports);
    }
};

class Divide : public MulDiv
{
public:
    Divide(std::string given_name, double initial=1.0) :
        MulDiv(given_name, "*/", initial) {}
};

class Integrator : public Base
{
protected:
    Value _value;

public:
    Integrator(std::string given_name, Value ic=Value(0.0)) : Base(given_name), _value(ic) {}

    void get_states(StatesInfo& states) override
    {
        states.add(_oports.front(), Value(_value), _iports.front());
    }

    void step(double /*t*/, const Values& values) override
    {
        assert(values.get(_oports.front()));
        _value = *values.get(_oports.front());
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
    Delay(std::string given_name, double lifespan=10.0) : Base(given_name), _lifespan(lifespan) {}

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
    Memory(std::string given_name, const Value& ic=Value::Zero(1)) :
        Base(given_name), _value(ic) {}

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
    Derivative(std::string given_name, const Value& y0=Value::Zero(1)) :
        Base(given_name), _y(y0) {}

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
    std::vector<Base*> _components;

    Parent(std::string given_name) : Base(given_name) {}

public:
    bool add_block(Base& component, const Signals& iports={}, const Signals& oports={})
    {
        if (!component.init(*this, iports, oports))
            return false;

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
        for (auto* component: _components)
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
    Submodel(std::string given_name) : Parent(given_name) {}

}; // class Submodel

class Model : public Parent
{
protected:
    std::vector<std::string> _registered_signals;

    bool init(Parent&, const Signals& = {}, const Signals& = {}) override;

public:
    Model(std::string given_name="model");

    Model* model() override {return this;}
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
    auto* model_ = model();
    return model_ ? Signal(given_name, *model_) : Signal();
}

}

#endif // __BLOCKS_HPP__
