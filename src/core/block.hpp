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
    bool _initialized{false};
    Signals _iports;
    Signals _oports;
    Signals _dependencies;
    Parent* _parent{nullptr};
    std::string _given_name;
    std::string _full_name;
    uint16_t _num_iports{NoIOLimit};
    uint16_t _num_oports{NoIOLimit};
    std::size_t _unnamed_signal_counter{0};

    bool _processed{false};
    void _assign_valid_given_name(std::string given_name);
    bool _add_dependecny(Signal signal);

    Block(std::string given_name, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit) :
        _given_name(given_name), _num_iports(num_iports), _num_oports(num_oports) {}

    virtual bool init(Parent& parent, const Signals& iports={}, const Signals& oports={});
    virtual void post_init() {}

public:
    virtual ~Block() = default;

    virtual void step(double /*t*/, const Values& /*states*/) {}
    virtual void activation_function(double /*t*/, Values& /*x*/) {}
    virtual Model* model();

    Model& model_ref()
    {
        auto* mdl = model();
        verify(mdl, _full_name + ": a model is necessary but none is defined!");
        return *mdl;
    }

    Parent* parent() {return _parent;}
    bool processed() const {return _processed;}
    bool is_initialized() const {return _initialized;}
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

        _components.push_back(&component);
        component.post_init();
        return true;
    }

    void step(double t, const Values& values) override
    {
        for (auto* component: _components)
            component->step(t, values);
    }

    ScalarSignal signal(const std::string& given_name="");
    ArraySignal  signal(const std::string& given_name, std::size_t size);
    template<typename Iter>
    BusSignal    signal(const std::string& given_name, const BusSpec& spec, Iter begin_, Iter end_);
    BusSignal    signal(const std::string& given_name, const BusSpec& spec, const std::initializer_list<BusSignalInfo::NameSignal>& l);
    BusSignal    signal(const std::string& given_name, const BusSpec& spec, const std::initializer_list<Signal>& l);

    Signal       clone_signal(const std::string& given_name, Signal sig);
    ScalarSignal clone_signal(const std::string& given_name, ScalarSignal sig)
    {
        return clone_signal(given_name, Signal(sig))->as_scalar();
    }
    ArraySignal clone_signal(const std::string& given_name, ArraySignal sig)
    {
        return clone_signal(given_name, Signal(sig))->as_array();
    }
    BusSignal clone_signal(const std::string& given_name, BusSignal sig)
    {
        return clone_signal(given_name, Signal(sig))->as_bus();
    }
    BusSignal bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<BusSignalInfo::NameSignal>& l)
    {
        return signal(given_name, spec, l);
    }
    BusSignal bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<Signal>& l)
    {
        return signal(given_name, spec, l);
    }

    std::string make_signal_name(const std::string& given_name);
    ScalarSignal parameter(const std::string& given_name);
    ArraySignal  parameter(const std::string& given_name, std::size_t size);
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

    virtual void input_cb(double /*t*/, Values& /*values*/) {}
};

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
            _value = values.get<T>(_iports[0]);
            _iports.clear();
            _init = false;
        }
        values.set<T>(_oports[0], _value);
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
        values.set<T>(_oports[0], _value);
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
        values.set<T>(_oports[0], _k * values.get<T>(_iports[0]));
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
        values.set<T>(_oports[0], values.get<T>(_iports[0]).sin());
    }
};

template<>
void SinT<double>::activation_function(double, Values&);

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
        values.set<T>(_oports[0], _act_func(t, values.get<T>(_iports[0])));
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
    T                 _ret;

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
        _ret = _initial;
        const char* p = _operators.c_str();
        for (const auto& signal: _iports)
        {
            const auto &v = values.get<T>(signal);
            if (*p == '+')
                _ret += v;
            else if (*p == '-')
                _ret -= v;
            else
                 assert(false);
            p++;
        }
        values.set<T>(_oports[0], _ret);
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
    T                 _ret;

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
        _ret = _initial;
        const char* p = _operators.c_str();
        for (const auto& signal: _iports)
        {
            const auto &v = values.get<T>(signal);
            if (*p == '*')
                _ret *= v;
            else if (*p == '/')
                _ret /= v;
            else
                 assert(false);
            p++;
        }
        values.set<T>(_oports[0], _ret);
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

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Block::init(parent, iports, oports))
            return false;

        model_ref().signal_registry().register_state(_oports[0], _iports[0], _value);

        return true;
    }

    void step(double /*t*/, const Values& values) override
    {
        assert(values.valid(_oports[0]));
        _value = values.get<T>(_iports[0]);
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

    // input signals
    typename pooya::Types<T>::Signal _s_x;       // [0]
    pooya::ScalarSignal              _s_delay;   // [1]
    typename pooya::Types<T>::Signal _s_initial; // [2]

    // output signal
    typename pooya::Types<T>::Signal _s_y; // [0]

public:
    DelayT(std::string given_name, double lifespan=10.0) : Block(given_name, 3, 1), _lifespan(lifespan) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Block::init(parent, iports, oports))
            return false;

        // input signals
        _iports.bind(0, _s_x);
        _iports.bind(1, _s_delay);
        _iports.bind(2, _s_initial);

        // output signal
        _oports.bind(0, _s_y);

        return true;
    }

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

        assert(_t.empty() || (t > _t.back()));
        _t.push_back(t);
        _x.push_back(values.get(_s_x));
    }

    void activation_function(double t, Values& values) override
    {
        if (_t.empty())
        {
            values.set(_s_y, values.get(_s_initial));
            return;
        }
    
        double delay = values.get(_s_delay);
        t -= delay;
        if (t <= _t[0])
        {
            values.set(_s_y, values.get(_s_initial));
        }
        else if (t >= _t.back())
        {
            values.set(_s_y, _x.back());
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

            values.set(_s_y, (_x[k] - _x[k - 1])*(t - _t[k - 1])/(_t[k] - _t[k - 1]) + _x[k - 1]);
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
        _value = values.get<T>(_iports[0]);
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

        values.set<T>(_oports[0], _value);
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
        _x = values.get<T>(_iports[0]);
        _y = _x;
        _first_step = false;
    }

    void activation_function(double t, Values& values) override
    {
        if (_first_step)
        {
            _t = t;
            _x = values.get<T>(_iports[0]);
            values.set<T>(_oports[0], _y);
        }
        else if (_t == t)
        {
            values.set<T>(_oports[0], _y);
        }
        else
        {
            values.set<T>(_oports[0], (values.get<T>(_iports[0]) - _x)/(t - _t));
        }
    }
};

using Derivative  = DerivativeT<double>;
using DerivativeA = DerivativeT<Array>;

class BusBlockBuilder : public Block
{
protected:
    BusSignal _x;
    BusSignal _y;

    std::vector<Block*> _blocks;

    void traverse_bus(const std::string& path, const BusSpec& bus_spec);

    virtual void block_builder(const std::string& path, const BusSpec::WireInfo& wi, Signal sig_in, Signal sig_out) = 0;

public:
    BusBlockBuilder(std::string given_name) :
        Block(given_name, 1, 1) {}

    ~BusBlockBuilder() override;

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override;
    void post_init() override;
};

inline ScalarSignal Parent::parameter(const std::string& given_name)
{
    auto* model_ = model();
    return model_ ? model_->signal(given_name) : nullptr;
}

inline ArraySignal Parent::parameter(const std::string& given_name, std::size_t size)
{
    auto* model_ = model();
    return model_ ? model_->signal(given_name, size) : nullptr;
}

}

#endif // __POOYA_BLOCK_HPP__
