/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_BLOCK_LIB_HPP__
#define __POOYA_BLOCK_LIB_HPP__

#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <map>
#include <memory>

#include "block_base.hpp"
#include "util.hpp"

namespace pooya
{

template <typename T>
class InitialValueT : public SingleInputOutputT<T>
{
protected:
    T _value;
    bool _init{true};

public:
    InitialValueT(const std::string& given_name) : SingleInputOutputT<T>(given_name, 1, 1) {}

    void activation_function(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        if (_init)
        {
            _value = values.get(SingleInputOutputT<T>::_s_in);
            SingleInputOutputT<T>::_ibus = nullptr;
            _init = false;
        }
        values.set(SingleInputOutputT<T>::_s_out, _value);
    }
};

using InitialValue = InitialValueT<double>;
using InitialValueA = InitialValueT<Array>;

template <typename T>
class ConstT : public SingleOutputT<T>
{
protected:
    T _value;

public:
    ConstT(const std::string& given_name, const T &value)
            : SingleOutputT<T>(given_name, 0, 1), _value(value) {}

    void activation_function(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        values.set(SingleOutputT<T>::_s_out, _value);
    }
};

using Const = ConstT<double>;
using ConstA = ConstT<Array>;

template <typename T, typename GainType>
class GainT : public SingleInputOutputT<T>
{
protected:
    GainType _k;

public:
    GainT(const std::string& given_name, GainType k) : SingleInputOutputT<T>(given_name, 1, 1), _k(k) {}

    void activation_function(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        values.set(SingleInputOutputT<T>::_s_out, _k * values.get(SingleInputOutputT<T>::_s_in));
    }

    typename Types<GainType>::GetValue gain() const {return _k;}
};

using Gain = GainT<double, double>;
using GainI = GainT<int, int>;
using GainA = GainT<Array, double>;

template <typename T>
class SinT : public SingleInputOutputT<T>
{
public:
    SinT(const std::string& given_name) : SingleInputOutputT<T>(given_name, 1, 1) {}

    void activation_function(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        values.set(SingleInputOutputT<T>::_s_out, values.get(SingleInputOutputT<T>::_s_in).sin());
    }
};

template<>
void SinT<double>::activation_function(double, Values &);

using Sin = SinT<double>;
using SinA = SinT<Array>;

template <typename T>
class SISOFunctionT : public SingleInputOutputT<T>
{
public:
    using ActFunction = std::function<typename Types<T>::SetValue(double, typename Types<T>::GetValue)>;

protected:
    ActFunction _act_func;

public:
    SISOFunctionT(const std::string& given_name, ActFunction act_func)
            : SingleInputOutputT<T>(given_name, 1, 1), _act_func(act_func) {}

    void activation_function(double t, Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        values.set(SingleInputOutputT<T>::_s_out, _act_func(t, values.get(SingleInputOutputT<T>::_s_in)));
    }
};

using SISOFunction = SISOFunctionT<double>;
using SISOFunctionA = SISOFunctionT<Array>;

template <typename T>
class SOFunctionT : public SingleOutputT<T>
{
public:
    using ActFunction = std::function<typename Types<T>::SetValue(double, BusId ibus, const Values&)>;

protected:
    ActFunction _act_func;

public:
    SOFunctionT(const std::string& given_name, ActFunction act_func, uint16_t num_iports=Block::NoIOLimit)
            : SingleOutputT<T>(given_name, num_iports, 1), _act_func(act_func) {}

    void activation_function(double t, Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        values.set(SingleOutputT<T>::_s_out, _act_func(t, SingleOutputT<T>::_ibus, values));
    }
};

using SOFunction = SOFunctionT<double>;
using SOFunctionA = SOFunctionT<Array>;

class Function : public Block
{
public:
    using ActFunction = std::function<void(double, BusId ibus, BusId obus, Values&)>;

protected:
    ActFunction _act_func;

public:
    Function(const std::string& given_name, ActFunction act_func, uint16_t num_iports=NoIOLimit, uint16_t num_oports=NoIOLimit)
        : Block(given_name, num_iports, num_oports), _act_func(act_func) {}

    void activation_function(double t, Values &values) override
    {
        pooya_trace("block: " + full_name());
        _act_func(t, _ibus, _obus, values);
    }
};

template <typename T>
class SourceT : public SingleOutputT<T>
{
public:
    using SourceFunction = std::function<T(double)>;

protected:
    SourceFunction _src_func;

public:
    SourceT(const std::string& given_name, SourceFunction src_func) :
        SingleOutputT<T>(given_name, Block::NoIOLimit, 1), _src_func(src_func) {}

    void activation_function(double t, Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        values.set(SingleOutputT<T>::_s_out, _src_func(t));
    }
};

using Source = SourceT<double>;
using SourceI = SourceT<int>;
using SourceA = SourceT<Array>;

class Sources : public Block
{
public:
    using SourcesFunction = std::function<void(BusId, double, Values&)>;

protected:
    SourcesFunction _src_func;

public:
    Sources(const std::string& given_name, SourcesFunction src_func, uint16_t num_oports=NoIOLimit) :
        Block(given_name, 0, num_oports), _src_func(src_func) {}

    void activation_function(double t, Values &values) override
    {
        pooya_trace("block: " + full_name());
        _src_func(_obus, t, values);
    }
};

template <typename T>
class AddSubT : public SingleOutputT<T>
{
protected:
    std::string _operators;
    T _initial;
    T _ret;

    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (!SingleOutputT<T>::init(parent, ibus, obus))
            return false;

        pooya_verify_valid_signal(ibus);
        pooya_verify_valid_signal(obus);
        pooya_verify(ibus->size() >= 1,
                     SingleOutputT<T>::full_name() + " requires 1 or more input signals.");
        pooya_verify(_operators.size() == ibus->size(),
                     SingleOutputT<T>::full_name() + ": mismatch between input signals and operators.");

        return true;
    }

public:
    AddSubT(const std::string& given_name, const char *operators, const T &initial = 0.0)
            : SingleOutputT<T>(given_name, Block::NoIOLimit, 1), _operators(operators), _initial(initial) {}

    void activation_function(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        _ret = _initial;
        const char *p = _operators.c_str();
        for (const auto &ls: *SingleOutputT<T>::_ibus)
        {
            const auto &v = values.get<T>(ls.second);
            if (*p == '+')
                _ret += v;
            else if (*p == '-')
                _ret -= v;
            else
                assert(false);
            p++;
        }
        values.set(SingleOutputT<T>::_s_out, _ret);
    }
};

using AddSub = AddSubT<double>;
using AddSubA = AddSubT<Array>;

template <typename T>
class AddT : public AddSubT<T>
{
protected:
    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + AddSubT<T>::full_name());
        AddSubT<T>::_operators = std::string(ibus->size(), '+');
        return AddSubT<T>::init(parent, ibus, obus);
    }

public:
    AddT(const std::string& given_name, const T &initial = 0.0)
            : AddSubT<T>(given_name, "", initial) {}
};

using Add = AddT<double>;
using AddA = AddT<Array>;

template <typename T>
class SubtractT : public AddSubT<T>
{
public:
    SubtractT(const std::string& given_name, const T &initial = 0.0)
            : AddSubT<T>(given_name, "+-", initial) {}
};

using Subtract = SubtractT<double>;
using SubtractA = SubtractT<Array>;

template <typename T>
class MulDivT : public SingleOutputT<T>
{
protected:
    std::string _operators;
    T _initial;
    T _ret;

    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (!SingleOutputT<T>::init(parent, ibus, obus))
            return false;

        pooya_verify(ibus->size() >= 1, "MulDivT requires 1 or more input signals.");
        pooya_verify(_operators.size() == ibus->size(),
                     SingleOutputT<T>::full_name() + ": mismatch between input signals and operators.");

        return true;
    }

public:
    MulDivT(const std::string& given_name, const char *operators, const T &initial = 1.0)
            : SingleOutputT<T>(given_name, Block::NoIOLimit, 1), _operators(operators), _initial(initial) {}

    void activation_function(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        _ret = _initial;
        const char *p = _operators.c_str();
        for (const auto &ls: *SingleOutputT<T>::_ibus)
        {
            const auto &v = values.get<T>(ls.second);
            if (*p == '*')
                _ret *= v;
            else if (*p == '/')
                _ret /= v;
            else
                assert(false);
            p++;
        }
        values.set(SingleOutputT<T>::_s_out, _ret);
    }
};

using MulDiv = MulDivT<double>;
using MulDivA = MulDivT<Array>;

template <typename T>
class MultiplyT : public MulDivT<T>
{
protected:
    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + MulDivT<T>::full_name());
        MulDivT<T>::_operators = std::string(ibus->size(), '*');
        return MulDivT<T>::init(parent, ibus, obus);
    }

public:
    MultiplyT(const std::string& given_name, const T &initial = 1.0)
            : MulDivT<T>(given_name, "", initial) {}
};

using Multiply = MultiplyT<double>;
using MultiplyA = MultiplyT<Array>;

template <typename T>
class DivideT : public MulDivT<T>
{
public:
    DivideT(const std::string& given_name, const T &initial = 1.0)
            : MulDivT<T>(given_name, "*/", initial) {}
};

using Divide = DivideT<double>;
using DivideA = DivideT<Array>;

template <typename T>
class IntegratorBaseT : public SingleOutputT<T>
{
protected:
    T _value;

public:
    IntegratorBaseT(const std::string& given_name, T ic = T(0.0), uint16_t num_iports=Block::NoIOLimit, uint16_t num_oports=Block::NoIOLimit)
            : SingleOutputT<T>(given_name, num_iports, num_oports), _value(ic) {}

    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (!SingleOutputT<T>::init(parent, ibus, obus))
            return false;

        SingleOutputT<T>::model_ref().register_state_variable(SingleOutputT<T>::_s_out, Types<T>::as_type(SingleOutputT<T>::_ibus->at(0).second));

        return true;
    }

    void pre_step(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        assert(!values.valid(SingleOutputT<T>::_s_out));
        values.set(SingleOutputT<T>::_s_out, _value);
    }

    void post_step(double /*t*/, const Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        assert(values.valid(SingleOutputT<T>::_s_out));
        _value = values.get(SingleOutputT<T>::_s_out);
    }

    uint _process(double /*t*/, Values &values, bool /*go_deep*/ = true) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (SingleOutputT<T>::_processed)
            return 0;

        SingleOutputT<T>::_processed = values.valid(SingleOutputT<T>::_s_out);
        return SingleOutputT<T>::_processed ? 1 : 0; // is it safe to simply return _processed?
    }
};

template <typename T>
class IntegratorT : public IntegratorBaseT<T>
{
public:
    IntegratorT(const std::string& given_name, T ic = T(0.0)) : IntegratorBaseT<T>(given_name, ic, 1, 1) {}
};

using Integrator = IntegratorT<double>;
using IntegratorA = IntegratorT<Array>;

template <typename T>
class TriggeredIntegratorT : public IntegratorBaseT<T>
{
protected:
    BoolSignalId _trigger;
    bool _triggered{false};

public:
    TriggeredIntegratorT(const std::string& given_name, T ic = T(0.0)) : IntegratorBaseT<T>(given_name, ic, 2, 1) {}

    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + IntegratorBaseT<T>::full_name());
        if (!IntegratorBaseT<T>::init(parent, ibus, obus))
            return false;

        _trigger = IntegratorBaseT<T>::bool_input_at("trigger");

        return true;
    }

    void pre_step(double t, Values &values) override
    {
        pooya_trace("block: " + IntegratorBaseT<T>::full_name());
        if (_triggered)
        {
            if constexpr (std::is_same_v<T, Array>)
                IntegratorBaseT<T>::_value.setZero();
            else
                IntegratorBaseT<T>::_value = 0;
            _triggered = false;
        }
        IntegratorBaseT<T>::pre_step(t, values);
    }

    uint _process(double t, Values &values, bool go_deep=true) override
    {
        pooya_trace("block: " + IntegratorBaseT<T>::full_name());
        if (IntegratorBaseT<T>::_processed || !values.valid(_trigger))
            return 0;

        if (!_triggered)
            _triggered = values.get(_trigger);

        return IntegratorBaseT<T>::_process(t, values, go_deep);
    }
};

using TriggeredIntegrator = TriggeredIntegratorT<double>;
using TriggeredIntegratorA = TriggeredIntegratorT<Array>;

template <typename T>
class DelayT : public SingleOutputT<T>
{
protected:
    double _lifespan;
    std::vector<double> _t;
    std::vector<T> _x;

    // input signals
    typename Types<T>::SignalId _s_x;       // in
    ScalarSignalId _s_delay;                // delay
    typename Types<T>::SignalId _s_initial; // initial

public:
    DelayT(const std::string& given_name, double lifespan = 10.0)
            : SingleOutputT<T>(given_name, 3, 1), _lifespan(lifespan) {}

    bool init(Parent &parent, BusId ibus, BusId obus) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (!SingleOutputT<T>::init(parent, ibus, obus))
            return false;

        // input signals
        _s_x = Types<T>::as_type(SingleOutputT<T>::_ibus->at("in"));
        _s_delay = SingleOutputT<T>::scalar_input_at("delay");
        _s_initial = Types<T>::as_type(SingleOutputT<T>::_ibus->at("initial"));

        return true;
    }

    void post_step(double t, const Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (!_t.empty())
        {
            double t1 = t - _lifespan;
            int k = 0;
            for (const auto &v : _t)
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

    void activation_function(double t, Values &values) override
    {
        pooya_trace("block: " + SingleOutputT<T>::full_name());
        if (_t.empty())
        {
            values.set(SingleOutputT<T>::_s_out, values.get(_s_initial));
            return;
        }

        double delay = values.get(_s_delay);
        t -= delay;
        if (t <= _t[0])
        {
            values.set(SingleOutputT<T>::_s_out, values.get(_s_initial));
        }
        else if (t >= _t.back())
        {
            values.set(SingleOutputT<T>::_s_out, _x.back());
        }
        else
        {
            int k = 0;
            for (const auto &v : _t)
            {
                if (v >= t)
                    break;
                k++;
            }

            values.set(SingleOutputT<T>::_s_out, (_x[k] - _x[k - 1]) * (t - _t[k - 1]) / (_t[k] - _t[k - 1]) + _x[k - 1]);
        }
    }
};

using Delay = DelayT<double>;
using DelayA = DelayT<Array>;

template <typename T>
class MemoryT : public SingleInputOutputT<T>
{
protected:
    T _value;

public:
    MemoryT(const std::string& given_name, const T &ic = T(0))
            : SingleInputOutputT<T>(given_name, 1, 1), _value(ic) {}

    void post_step(double /*t*/, const Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        _value = values.get(SingleInputOutputT<T>::_s_in);
    }

    // # Memory can be implemented either by defining the following activation
    // function #   (which is more straightforward) or through overloading the
    // _process method #   which is more efficient since it deosn't rely on the
    // input signal being known. #   Both approaches are supposed to lead to the
    // exact same results.

    // # def activation_function(self, t, x):
    // #     return [self._value]

    uint _process(double /*t*/, Values &values, bool /*go_deep*/) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        if (SingleInputOutputT<T>::_processed)
            return 0;

        pooya_trace_update0;
        values.set(SingleInputOutputT<T>::_s_out, _value);
        SingleInputOutputT<T>::_processed = true;
        return 1;
    }
};

using Memory = MemoryT<double>;
using MemoryI = MemoryT<int>;
using MemoryB = MemoryT<bool>;
using MemoryA = MemoryT<Array>;

template <typename T>
class DerivativeT : public SingleInputOutputT<T>
{
protected:
    bool _first_step{true};
    double _t;
    T _x;
    T _y;

public:
    DerivativeT(const std::string& given_name, const T &y0 = 0)
            : SingleInputOutputT<T>(given_name, 1, 1), _y(y0) {}

    void post_step(double t, const Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        _t = t;
        _x = values.get(SingleInputOutputT<T>::_s_in);
        _y = _x;
        _first_step = false;
    }

    void activation_function(double t, Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        if (_first_step)
        {
            _t = t;
            _x = values.get(SingleInputOutputT<T>::_s_in);
            values.set(SingleInputOutputT<T>::_s_out, _y);
        }
        else if (_t == t)
        {
            values.set(SingleInputOutputT<T>::_s_out, _y);
        }
        else
        {
            values.set(SingleInputOutputT<T>::_s_out, (values.get(SingleInputOutputT<T>::_s_in) - _x) / (t - _t));
        }
    }
};

using Derivative = DerivativeT<double>;
using DerivativeA = DerivativeT<Array>;

template <typename T>
class PipeT : public SingleInputOutputT<T>
{
public:
    explicit PipeT(const std::string& given_name) : SingleInputOutputT<T>(given_name, 1, 1) {}

    void activation_function(double /*t*/, Values &values) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name());
        values.set(SingleInputOutputT<T>::_s_out, values.get(SingleInputOutputT<T>::_s_in));
    }
};

using Pipe = PipeT<double>;
using PipeI = PipeT<int>;
using PipeB = PipeT<bool>;
using PipeA = PipeT<Array>;

class BusBlockBuilder : public SingleInputOutputT<BusSpec>
{
protected:
    std::vector<std::unique_ptr<Block>> _blocks;
    std::vector<std::string> _excluded_labels;

    void traverse_bus(const std::string &path_name, const BusSpec &bus_spec);

    virtual void block_builder(const std::string &path_name, const BusSpec::WireInfo &wi,
        SignalId sig_in, SignalId sig_out) = 0;

public:
    BusBlockBuilder(const std::string& given_name, const std::initializer_list<std::string>& excluded_labels={})
        : SingleInputOutputT<BusSpec>(given_name, 1, 1), _excluded_labels(excluded_labels) {}

    bool init(Parent &parent, BusId ibus, BusId obus) override;
    void post_init() override;
};

class BusMemory : public BusBlockBuilder
{
public:
    using LabelValueMap = std::map<std::string, Value>;
    using LabelValue = LabelValueMap::value_type;

protected:
    LabelValueMap _init_values;

public:
    BusMemory(const std::string& given_name, const std::initializer_list<LabelValue> &l = {}, const std::initializer_list<std::string>& excluded_labels={})
            : BusBlockBuilder(given_name, excluded_labels), _init_values(l) {}

protected:
    void block_builder(const std::string &full_label, const BusSpec::WireInfo &wi, SignalId sig_in, SignalId sig_out) override;
    void post_init() override;
};

class BusPipe : public BusBlockBuilder
{
public:
    explicit BusPipe(const std::string& given_name, const std::initializer_list<std::string>& excluded_labels={})
            : BusBlockBuilder(given_name, excluded_labels) {}

protected:
    void block_builder(const std::string & /*full_label*/, const BusSpec::WireInfo &wi,
        SignalId sig_in, SignalId sig_out) override;
};

} // namespace pooya

#endif // __POOYA_BLOCK_LIB_HPP__
