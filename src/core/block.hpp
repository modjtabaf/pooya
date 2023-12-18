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

#ifndef __POOYA_BLOCK_HPP__
#define __POOYA_BLOCK_HPP__

#include <cassert>
#include "util.hpp"
#include "signal.hpp"

namespace pooya
{

class Block;
class Parent;
class Model;

using TraverseCallback = std::function<bool(Block&, uint32_t level)>;

class Block
{
    friend class Parent;

public:
    static constexpr uint16_t NoIOLimit = uint16_t(-1);

protected:
    Signals _iports;
    Signals _oports;
    Signals _dependencies;
    Parent* _parent{nullptr};
    std::string _given_name;
    std::string _full_name;
    uint16_t _num_iports{NoIOLimit};
    uint16_t _num_oports{NoIOLimit};

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);
    bool _add_dependecny(const Signal& signal);

    Block(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) :
        _given_name(given_name), _num_iports(num_iports), _num_oports(num_oports) {}

    virtual bool init(Parent& parent, const Signals& iports={}, const Signals& oports={});

public:
    virtual ~Block() = default;

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
}; // class Block

template<typename T>
class InitialValueT : public Block
{
protected:
    T _value;
    bool _init{true};

public:
    InitialValueT(std::string given_name) : Block(given_name, 1, 1) {}

    void activation_function(double /*t*/, Values& values) override
    {
        if (_init)
        {
            _value = get_input(T, 0);
            _iports.clear();
            _init = false;
        }
        set_output(T, 0, _value);
    }
};

using InitialValue  = InitialValueT<double>;
using InitialValueA = InitialValueT<Array>;

template<typename T>
class ConstT : public Block
{
protected:
    T _value;

public:
    ConstT(std::string given_name, const T& value) : Block(given_name, 0, 1), _value(value) {}

    void activation_function(double /*t*/, Values& values) override
    {
        set_output(T, 0, _value);
    }
};

using Const  = ConstT<double>;
using ConstA = ConstT<Array>;

template<typename T>
class GainT : public Block
{
protected:
    double _k;

public:
    GainT(std::string given_name, double k) : Block(given_name, 1, 1), _k(k) {}

    void activation_function(double /*t*/, Values& values) override
    {
        set_output(T, 0, _k * get_input(T, 0));
    }
};

using Gain  = GainT<double>;
using GainA = GainT<Array>;

template<typename T>
class SinT : public Block
{
public:
    SinT(std::string given_name) : Block(given_name, 1, 1) {}

    void activation_function(double /*t*/, Values& values) override
    {
        if constexpr (std::is_same_v<T, double>)
        {
            double x = scalar_input(0);
            scalar_output(0, std::sin(x));
        }
        else
        {
            const T& x = array_input(0);
            array_output(0, x.sin());
        }
    }
};

using Sin  = SinT<double>;
using SinA = SinT<Array>;

template<typename T>
class FunctionT : public Block
{
public:
    using ActFunction = std::function<T(double, const T&)>;

protected:
    ActFunction _act_func;

public:
    FunctionT(std::string given_name, ActFunction act_func) :
        Block(given_name, 1, 1), _act_func(act_func) {}

    void activation_function(double t, Values& values) override
    {
        set_output(T, 0, _act_func(t, get_input(T, 0)));
    }
};

using Function  = FunctionT<double>;
using FunctionA = FunctionT<Array>;

template<typename T>
class AddSubT : public Block
{
protected:
    std::string _operators;
    T             _initial;

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Block::init(parent, iports, oports))
            return false;

        verify(iports.size() >= 1, _full_name + " requires 1 or more input signals.");
        verify(_operators.size() == iports.size(), _full_name + ": mismatch between input signals and operators.");

        return true;
    }

public:
    AddSubT(std::string given_name, const char* operators, const T& initial=0.0) :
        Block(given_name, NoIOLimit, 1), _operators(operators), _initial(initial)
    {}

    void activation_function(double /*t*/, Values& values) override
    {
        T ret = _initial;
        const char* p = _operators.c_str();
        for (const auto& signal: _iports)
        {
            const auto &v = values.get<T>(signal);
            if (*p == '+')
                ret += v;
            else if (*p == '-')
                ret -= v;
            else
                 assert(false);
            p++;
        }
        set_output(T, 0, ret);
    }
};

using AddSub  = AddSubT<double>;
using AddSubA = AddSubT<Array>;

template<typename T>
class AddT : public AddSubT<T>
{
protected:
    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        AddSubT<T>::_operators = std::string(iports.size(), '+');
        return AddSubT<T>::init(parent, iports, oports);
    }

public:
    AddT(std::string given_name, const T& initial=0.0) :
        AddSubT<T>(given_name, "", initial) {}
};

using Add  = AddT<double>;
using AddA = AddT<Array>;

template<typename T>
class SubtractT : public AddSubT<T>
{
public:
    SubtractT(std::string given_name, const T& initial=0.0) :
        AddSubT<T>(given_name, "+-", initial) {}
};

using Subtract  = SubtractT<double>;
using SubtractA = SubtractT<Array>;

template<typename T>
class MulDivT : public Block
{
protected:
    std::string _operators;
    T             _initial;

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Block::init(parent, iports, oports))
            return false;

        verify(iports.size() >= 1, "MulDivT requires 1 or more input signals.");
        verify(_operators.size() == iports.size(), _full_name + ": mismatch between input signals and operators.");

        return true;
    }

public:
    MulDivT(std::string given_name, const char* operators, const T& initial=1.0) :
        Block(given_name, NoIOLimit, 1), _operators(operators), _initial(initial) {}

    void activation_function(double /*t*/, Values& values) override
    {
        T ret = _initial;
        const char* p = _operators.c_str();
        for (const auto& signal: _iports)
        {
            const auto &v = values.get<T>(signal);
            if (*p == '*')
                ret *= v;
            else if (*p == '/')
                ret /= v;
            else
                 assert(false);
            p++;
        }
        set_output(T, 0, ret);
    }
};

using MulDiv  = MulDivT<double>;
using MulDivA = MulDivT<Array>;

template<typename T>
class MultiplyT : public MulDivT<T>
{
protected:
    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        MulDivT<T>::_operators = std::string(iports.size(), '*');
        return MulDivT<T>::init(parent, iports, oports);
    }

public:
    MultiplyT(std::string given_name, const T& initial=1.0) :
        MulDivT<T>(given_name, "", initial) {}
};

using Multiply  = MultiplyT<double>;
using MultiplyA = MultiplyT<Array>;

template<typename T>
class DivideT : public MulDivT<T>
{
public:
    DivideT(std::string given_name, const T& initial=1.0) :
        MulDivT<T>(given_name, "*/", initial) {}
};

using Divide  = DivideT<double>;
using DivideA = DivideT<Array>;

template<typename T>
class IntegratorT : public Block
{
protected:
    T _value;

public:
    IntegratorT(std::string given_name, T ic=T(0.0)) : Block(given_name, 1, 1), _value(ic) {}

    void get_states(StatesInfo& states) override
    {
        states.add(_oports[0], _value, _iports[0]);
    }

    void step(double /*t*/, const Values& values) override
    {
        assert(values.valid(_oports[0]));
        _value = get_input(T, 0);
    }

    uint _process(double /*t*/, Values& values, bool /*go_deep*/ = true) override
    {
        if (_processed)
            return 0;

        _processed = values.valid(_iports[0]);
        return _processed ? 1 : 0; // is it safe to simply return _processed?
    }
};

using Integrator  = IntegratorT<double>;
using IntegratorA = IntegratorT<Array>;

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

template<typename T>
class DelayT : public Block
{
protected:
    double  _lifespan;
    std::vector<double> _t;
    std::vector<T> _x;

public:
    DelayT(std::string given_name, double lifespan=10.0) : Block(given_name, 3, 1), _lifespan(lifespan) {}

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
        _x.push_back(get_input(T, 0));
    }

    void activation_function(double t, Values& values) override
    {
        if (_t.empty())
        {
            set_output(T, 0, get_input(T, 2));
            return;
        }
    
        double delay = scalar_input(1);
        t -= delay;
        if (t <= _t[0])
        {
            set_output(T, 0, get_input(T, 2));
        }
        else if (t >= _t.back())
        {
            set_output(T, 0, _x.back());
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

            set_output(T, 0, (_x[k] - _x[k - 1])*(t - _t[k - 1])/(_t[k] - _t[k - 1]) + _x[k - 1]);
        }
    }
};

using Delay  = DelayT<double>;
using DelayA = DelayT<Array>;

template<typename T>
class MemoryT : public Block
{
protected:
    T _value;

public:
    MemoryT(std::string given_name, const T& ic=0) :
        Block(given_name, 1, 1), _value(ic) {}

    void step(double /*t*/, const Values& values) override
    {
        _value = get_input(T, 0);
    }

    // # Memory can be implemented either by defining the following activation function
    // #   (which is more straightforward) or through overloading the _process method
    // #   which is more efficient since it deosn't rely on the input signal being known.
    // #   Both approaches are supposed to lead to the exact same results.

    // # def activation_function(self, t, x):
    // #     return [self._value]

    uint _process(double /*t*/, Values& values, bool /*go_deep*/) override
    {
        if (_processed)
            return 0;

        set_output(T, 0, _value);
        _processed = true;
        return 1;
    }
};

using Memory  = MemoryT<double>;
using MemoryA = MemoryT<Array>;

template<typename T>
class DerivativeT : public Block
{
protected:
    bool _first_step{true};
    double _t;
    T _x;
    T _y;

public:
    DerivativeT(std::string given_name, const T& y0=0) :
        Block(given_name, 1, 1), _y(y0) {}

    void step(double t, const Values& values) override
    {
        _t = t;
        _x = get_input(T, 0);
        _y = _x;
        _first_step = false;
    }

    void activation_function(double t, Values& values) override
    {
        if (_first_step)
        {
            _t = t;
            _x = get_input(T, 0);
            set_output(T, 0, _y);
        }
        else if (_t == t)
        {
            set_output(T, 0, _y);
        }
        else
        {
            set_output(T, 0, (get_input(T, 0) - _x)/(t - _t));
        }
    }
};

using Derivative  = DerivativeT<double>;
using DerivativeA = DerivativeT<Array>;

class Parent : public Block
{
protected:
    std::vector<Block*> _components;

    Parent(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) :
        Block(given_name, num_iports, num_oports) {}

public:
    bool add_block(Block& component, const Signals& iports={}, const Signals& oports={})
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

    Signal signal(const std::string& given_name="", std::size_t size=0)
    {
        return Signal(given_name, *this, size);
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
    Submodel(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) : Parent(given_name, num_iports, num_oports) {}

}; // class Submodel

class Model : public Parent
{
protected:
    SignalRegistry _signal_registry;

    bool init(Parent&, const Signals& = {}, const Signals& = {}) override;

public:
    Model(std::string given_name="model");

    Model* model() override {return this;}

    const SignalRegistry& signal_registry() const {return _signal_registry;}
    SignalRegistry& signal_registry() {return _signal_registry;}

    Signal::Id find_signal(const std::string& name, bool exact_match = false) const
    {
        return _signal_registry.find_signal(name, exact_match);
    }
    Signal::Id register_signal(const std::string& name, std::size_t size)
    {
        return _signal_registry.register_signal(name, size);
    }

    virtual void input_cb(double /*t*/, Values& /*values*/) {}
};

inline Signal Parent::parameter(const std::string& given_name)
{
    auto* model_ = model();
    return model_ ? Signal(given_name, *model_) : Signal();
}

}

#endif // __POOYA_BLOCK_HPP__
