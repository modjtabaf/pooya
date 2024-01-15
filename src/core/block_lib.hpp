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

#ifndef __POOYA_BLOCK_LIB_HPP__
#define __POOYA_BLOCK_LIB_HPP__

#include "block_base.hpp"

namespace pooya
{

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

        model_ref().register_state(_oports[0], _iports[0], _value);

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

template<typename T>
class DelayT : public Block
{
protected:
    double  _lifespan;
    std::vector<double> _t;
    std::vector<T> _x;

    // input signals
    typename Types<T>::Signal _s_x;       // [0]
    ScalarSignal              _s_delay;   // [1]
    typename Types<T>::Signal _s_initial; // [2]

    // output signal
    typename Types<T>::Signal _s_y; // [0]

public:
    DelayT(std::string given_name, double lifespan=10.0) : Block(given_name, 3, 1), _lifespan(lifespan) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Block::init(parent, iports, oports))
            return false;

        // input signals
        _iports.bind(_s_x);
        _iports.bind(_s_delay);
        _iports.bind(_s_initial);

        // output signal
        _oports.bind(_s_y);

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

    void traverse_bus(const std::string& path_name, const BusSpec& bus_spec);

    virtual void block_builder(const std::string& path_name, const BusSpec::WireInfo& wi, Signal sig_in, Signal sig_out) = 0;

public:
    BusBlockBuilder(std::string given_name) :
        Block(given_name, 1, 1) {}

    ~BusBlockBuilder() override;

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override;
    void post_init() override;
};

class BusMemory : public BusBlockBuilder
{
public:
    using LabelValue = std::pair<std::string, Value>;

protected:
    const std::vector<LabelValue> _init_values;

public:
    BusMemory(std::string given_name, const std::initializer_list<LabelValue>& l={}) : BusBlockBuilder(given_name), _init_values(l) {}

protected:
    void block_builder(const std::string& full_label, const BusSpec::WireInfo& wi, Signal sig_in, Signal sig_out) override
    {
        auto it = std::find_if(_init_values.begin(), _init_values.end(),
            [&](const LabelValue& lv)
            {
                return lv.first == full_label;
            });

        Block* block = nullptr;
        if (wi._scalar)
        {
            block = (it == _init_values.end()) ?
                new Memory("memory") :
                new Memory("memory", it->second.as_scalar());
        }
        else if (wi._array_size > 0)
        {
            block = (it == _init_values.end()) ?
                new MemoryA("memory") :
                new MemoryA("memory", it->second.as_array());
        }
        else
        {
            verify(false, "cannot create a memory block for a non-value signal.");
        }

        _blocks.push_back(block);
        _parent->add_block(*block, sig_in, sig_out);
    }
};

}

#endif // __POOYA_BLOCK_LIB_HPP__
