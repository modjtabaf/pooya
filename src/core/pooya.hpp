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

// helper macros

#define POOYA_HERE(s) std::cout << "Pooya:" << __FILE__ << ":" << __LINE__ << s << "\n" << std::flush;
// #undef POOYA_HERE
// #define POOYA_HERE

#define POOYA_HERE0 POOYA_HERE("")
// #undef POOYA_HERE0
// #define POOYA_HERE0

#define  get_input(T, index) values.get<T>(_iports[index])
#define  scalar_input(index) get_input(double, index)
#define   array_input(index) get_input( Value, index)

#define set_output(T, index, value) values.set<T>(_oports[index], value)
#define scalar_output(index, value) set_output(double, index, value)
#define  array_output(index, value) set_output( Value, index, value)

namespace pooya
{

#define verify(cond,msg) \
    if (!(cond)) \
        throw std::runtime_error( \
            std::string("\n" __FILE__ ":") + std::to_string(__LINE__) + "\n" + \
            "Pooya Exception: " + (msg) + "\n")

class Base;
class Parent;
class SignalRegistry;
class StatesInfo;
class Model;

template <int N=Eigen::Dynamic>
class ValueN : public Eigen::Array<double, N, 1>
{
public:
    using Parent = Eigen::Array<double, N, 1>;
    using Parent::Array;

    ValueN(double v=0) : Parent()
    {
        verify((N == 1) || (N == Eigen::Dynamic), "cannot initiaize an array with a scalar!");
        if constexpr (N == Eigen::Dynamic)
            Parent::resize(1);
        (*this)[0] = v;
    }
};

using Value = ValueN<>;

class Signal
{
public:
    using Id = std::size_t;
    static constexpr Id NoId = std::string::npos;

protected:
    std::string _given_name;
    std::string _full_name;
    std::size_t _id{NoId};
    std::size_t _size{0};

    std::string _make_valid_given_name(const std::string& given_name) const;
    void _set_owner(Parent& owner);

public:
    Signal(const std::string& given_name, Parent& owner, std::size_t size=0) : _given_name(_make_valid_given_name(given_name)), _size(size)
    {
        _set_owner(owner);
    }
    Signal(const char* given_name, Parent& owner, std::size_t size=0) : _given_name(_make_valid_given_name(given_name)), _size(size)
    {
        _set_owner(owner);
    }
    Signal(Parent& owner, std::size_t size=0) : _given_name(_make_valid_given_name("")), _size(size)
    {
        _set_owner(owner);
    }
    Signal() = default;
    Signal(const Signal& signal) = default;

    Signal& operator=(const Signal& rhs) = default;

    operator Id() const {return _id;}
    Id id() const {return _id;}
    std::size_t size() const {return _size;}
    const std::string& given_name() const {return _given_name;}
    const std::string& full_name() const {return _full_name;} // todo: create a shared parent with Base named NamedObject
};

inline std::ostream& operator<<(std::ostream& os, const Signal& signal)
{
    return os << "signal [" << signal.given_name() << "] = " << signal.id() << "\n";
}

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
public:
    struct ValueInfo
    {
        bool _assigned{false};
        std::size_t _id;
        std::size_t _start;
        std::size_t _size;
        bool _is_state;
        ValueInfo(Signal::Id id, std::size_t start, std::size_t size, bool is_state) :
            _id(id), _start(start), _size(size), _is_state(is_state) {}
        ValueInfo(Signal::Id id, std::size_t start, bool is_state) :
            _id(id), _start(start), _size(0), _is_state(is_state) {}
    };

protected:
    std::vector<ValueInfo> _values;
    std::size_t _total_size{0};
    Eigen::ArrayXd _array;
    Eigen::Map<Eigen::ArrayXd> _states{nullptr, Eigen::Dynamic};
    std::size_t _states_size{0};

    inline ValueInfo& get_value_info(Signal::Id id)
    {
        verify(id != Signal::NoId, "invalid id!");
        return _values[id];
    }

    template<typename T>
    auto _get(const ValueInfo& vi) const -> const auto;

    template<typename T>
    void _set(const ValueInfo& vi, const T& value)
    {
        verify(vi._size == std::size_t(value.rows()),
            std::string("size mismatch (id=") + std::to_string(vi._id) + ")(" + std::to_string(vi._size) +
            " vs " + std::to_string(value.rows()) + ")!");
        _array.segment(vi._start, vi._size) = value;
    }

public:
    // Values(const SignalRegistry& signal_registry);
    // Values(const StatesInfo& states);
    Values() {}
    Values(const SignalRegistry& signal_registry, const StatesInfo& states);

    inline const ValueInfo& get_value_info(Signal::Id id) const
    {
        verify(id != Signal::NoId, "invalid id!");
        return _values[id];
    }

    bool valid(Signal::Id id) const
    {
        return get_value_info(id)._assigned;
    }

    bool is_array(Signal::Id id) const
    {
        return get_value_info(id)._size > 0;
    }

    const decltype(_states)& states() const {return _states;}

    // Return type is VectorBlock<const Array<double, Eigen::Dynamic, 1>, Eigen::Dynamic>
    template<typename T>
    auto get(Signal::Id id) const -> const auto
    {
        const auto& vi = get_value_info(id);
        verify(vi._assigned, "attempting to access an unassigned value!");
        if constexpr (std::is_same_v<T, double>)
        {
            verify(vi._size == 0, "attempting to retrieve an array as a scalar!");
        }
        else
        {
            verify(vi._size > 0, "attempting to retrieve a scalar as an array!");
        }
        return _get<T>(vi);
    }

    double get_scalar(Signal::Id id) const;
    auto   get_array(Signal::Id id) const;

    template<typename T>
    void set(Signal::Id id, const T& value)
    {
        verify(id != Signal::NoId, "invalid id!");
        auto& vi = _values[id];
        verify(!vi._assigned, "re-assignment is prohibited!");
        _set<T>(vi, value);
        vi._assigned = true;
    }

    void set_scalar(Signal::Id id, double value);
    void set_array(Signal::Id id, const Value& value);

    void set_states(const Eigen::ArrayXd& states);

    void invalidate()
    {
        for (auto& v: _values) v._assigned = false;
    }

    void stream(std::ostream& os) const
    {
        int k = 0;
        for (const auto& v: _values)
        {
            os << "- [" << k++ << "]: ";
            (v._assigned ? os << _array.segment(v._start, v._size) : os << "*") << "\n";
        }
        os << "\n";
    }
};

template<>
inline auto Values::_get<Value>(const Values::ValueInfo& vi) const -> const auto
{
    return _array.segment(vi._start, vi._size);
}

template<>
inline auto Values::_get<double>(const Values::ValueInfo& vi) const -> const auto
{
    return _array[vi._start];
}

inline double Values::get_scalar(Signal::Id id) const {return get<double>(id);}
inline auto   Values::get_array (Signal::Id id) const {return get<Value> (id);}

template<>
inline void Values::_set<double>(const ValueInfo& vi, const double& value)
{
    verify(vi._size == 0, "cannot assign scalar to array!");
    _array(vi._start) = value;
}

inline void Values::set_scalar(Signal::Id id, double value) {set<double>(id, value);}
inline void Values::set_array (Signal::Id id, const Value& value) {set<Value>(id, value);}

inline std::ostream& operator<<(std::ostream& os, const Values& values)
{
    values.stream(os);
    return os;
}

struct StateInfo
{
    Signal::Id _id;
    Signal::Id _deriv_id;
    bool _scalar;
    Value _value;
    StateInfo(Signal::Id id, Signal::Id deriv_id, const Value& iv) :
        _id(id), _deriv_id(deriv_id), _scalar(false), _value(iv)
        {
        }
    StateInfo(Signal::Id id, Signal::Id deriv_id, double iv) :
        _id(id), _deriv_id(deriv_id), _scalar(true), _value(iv)
        {
        }
};

class StatesInfo : public std::vector<StateInfo>
{
protected:
    using Parent = std::vector<StateInfo>;

public:
    using Parent::vector;

    template<typename T=double>
    void add(Signal::Id state, const T& value, Signal::Id deriv)
    {
        verify(std::find_if(begin(), end(),
            [&] (const StateInfo& info) -> bool
            {
                return info._id == state;
            }) == end(), std::string("a state with id ") + std::to_string(state) + " is added already!");
        emplace_back(StateInfo(state, deriv, value));
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

template<typename T>
class InitialValueT : public Base
{
protected:
    T _value;
    bool _init{true};

public:
    InitialValueT(std::string given_name) : Base(given_name) {}

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
using InitialValueA = InitialValueT<Value>;

template<typename T>
class ConstT : public Base
{
protected:
    T _value;

public:
    ConstT(std::string given_name, const T& value) : Base(given_name), _value(value) {}

    void activation_function(double /*t*/, Values& values) override
    {
        set_output(T, 0, _value);
    }
};

using Const  = ConstT<double>;
using ConstA = ConstT<Value>;

template<typename T>
class GainT : public Base
{
protected:
    double _k;

public:
    GainT(std::string given_name, double k) : Base(given_name), _k(k) {}

    void activation_function(double /*t*/, Values& values) override
    {
        set_output(T, 0, _k * get_input(T, 0));
    }
};

using Gain  = GainT<double>;
using GainA = GainT<Value>;

template<typename T>
class SinT : public Base
{
public:
    SinT(std::string given_name) : Base(given_name) {}

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
using SinA = SinT<Value>;

template<typename T>
class FunctionT : public Base
{
public:
    using ActFunction = std::function<T(double, const T&)>;

protected:
    ActFunction _act_func;

public:
    FunctionT(std::string given_name, ActFunction act_func) :
        Base(given_name), _act_func(act_func) {}

    void activation_function(double t, Values& values) override
    {
        set_output(T, 0, _act_func(t, get_input(T, 0)));
    }
};

using Function  = FunctionT<double>;
using FunctionA = FunctionT<Value>;

template<typename T>
class AddSubT : public Base
{
protected:
    std::string _operators;
    T             _initial;

public:
    AddSubT(std::string given_name, const char* operators, const T& initial=0.0) :
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
using AddSubA = AddSubT<Value>;

template<typename T>
class AddT : public AddSubT<T>
{
public:
    AddT(std::string given_name, const T& initial=0.0) :
        AddSubT<T>(given_name, "", initial) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        AddSubT<T>::_operators = std::string(iports.size(), '+');
        return AddSubT<T>::init(parent, iports, oports);
    }
};

using Add  = AddT<double>;
using AddA = AddT<Value>;

template<typename T>
class SubtractT : public AddSubT<T>
{
public:
    SubtractT(std::string given_name, const T& initial=0.0) :
        AddSubT<T>(given_name, "+-", initial) {}
};

using Subtract  = SubtractT<double>;
using SubtractA = SubtractT<Value>;

template<typename T>
class MulDivT : public Base
{
protected:
    std::string _operators;
    T             _initial;

public:
    MulDivT(std::string given_name, const char* operators, const T& initial=1.0) :
        Base(given_name), _operators(operators), _initial(initial) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        assert(_operators.size() == iports.size());
        return (_operators.size() == iports.size()) &&
            Base::init(parent, iports, oports);
    }

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
using MulDivA = MulDivT<Value>;

template<typename T>
class MultiplyT : public MulDivT<T>
{
public:
    MultiplyT(std::string given_name, const T& initial=1.0) :
        MulDivT<T>(given_name, "", initial) {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        MulDivT<T>::_operators = std::string(iports.size(), '*');
        return MulDivT<T>::init(parent, iports, oports);
    }
};

using Multiply  = MultiplyT<double>;
using MultiplyA = MultiplyT<Value>;

template<typename T>
class DivideT : public MulDivT<T>
{
public:
    DivideT(std::string given_name, const T& initial=1.0) :
        MulDivT<T>(given_name, "*/", initial) {}
};

using Divide  = DivideT<double>;
using DivideA = DivideT<Value>;

template<typename T>
class IntegratorT : public Base
{
protected:
    T _value;

public:
    IntegratorT(std::string given_name, T ic=T(0.0)) : Base(given_name), _value(ic) {}

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
using IntegratorA = IntegratorT<Value>;

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
class DelayT : public Base
{
protected:
    double  _lifespan;
    Scalars _t;
    std::vector<T> _x;

public:
    DelayT(std::string given_name, double lifespan=10.0) : Base(given_name), _lifespan(lifespan) {}

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
using DelayA = DelayT<Value>;

template<typename T>
class MemoryT : public Base
{
protected:
    T _value;

public:
    MemoryT(std::string given_name, const T& ic=0) :
        Base(given_name), _value(ic) {}

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
using MemoryA = MemoryT<Value>;

template<typename T>
class DerivativeT : public Base
{
protected:
    bool _first_step{true};
    double _t;
    T _x;
    T _y;

public:
    DerivativeT(std::string given_name, const T& y0=0) :
        Base(given_name), _y(y0) {}

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
using DerivativeA = DerivativeT<Value>;

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
    Submodel(std::string given_name) : Parent(given_name) {}

}; // class Submodel

class SignalRegistry
{
public:
    using SignalInfo = std::pair<std::string, std::size_t>;
    using Signals = std::vector<SignalInfo>;

protected:
    Signals _signals;

    Signals::const_iterator _find_signal(const std::string& name, bool exact_match = false) const;

public:
    const Signals& signals() const {return _signals;}
    std::size_t num_signals() const {return _signals.size();}

    const SignalInfo& get_signal_by_id(Signal::Id id) const
    {
        return _signals[id];
    }

    Signal::Id find_signal(const std::string& name, bool exact_match = false) const;
    Signal::Id register_signal(const std::string& name, std::size_t size);
};

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

#endif // __BLOCKS_HPP__
