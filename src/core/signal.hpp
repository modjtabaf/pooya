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

#ifndef __POOYA_SIGNAL_HPP__
#define __POOYA_SIGNAL_HPP__

#include <memory>
#include "Eigen/Core"
#include "util.hpp"

namespace pooya
{

class Model;
class Parent;

template <int N=Eigen::Dynamic>
class ArrayN : public Eigen::Array<double, N, 1>
{
public:
    using _Parent = Eigen::Array<double, N, 1>;
    using _Parent::Array;

    ArrayN(double v=0) : _Parent()
    {
        verify((N == 1) || (N == Eigen::Dynamic), "cannot initiaize an array with a scalar!");
        if constexpr (N == Eigen::Dynamic)
            _Parent::resize(1);
        (*this)[0] = v;
    }
};

using Array = ArrayN<>;

class SignalRegistry;
class       SignalInfo;
class  ValueSignalInfo;
class ScalarSignalInfo;
class  ArraySignalInfo;
class    BusSignalInfo;

using       Signal = const       SignalInfo*;
using  ValueSignal = const  ValueSignalInfo*;
using ScalarSignal = const ScalarSignalInfo*;
using  ArraySignal = const  ArraySignalInfo*;
using    BusSignal = const    BusSignalInfo*;

class SignalInfo
{
    friend class SignalRegistry;

protected:
    ValueSignalInfo*   _value{nullptr};
    ScalarSignalInfo* _scalar{nullptr};
    ArraySignalInfo*   _array{nullptr};
    BusSignalInfo*       _bus{nullptr};

    SignalInfo(const std::string& full_name) : _full_name(full_name) {}

public:
    const std::string _full_name; // full name of the signal

    ValueSignal   as_value() const {return  _value;}
    ScalarSignal as_scalar() const {return _scalar;}
    ArraySignal   as_array() const {return  _array;}
    BusSignal       as_bus() const {return    _bus;}
};

class ValueSignalInfo : public SignalInfo
{
    friend class SignalRegistry;

protected:
    ValueSignal _deriv_sig{nullptr}; // the derivative signal if this is a state variable, nullptr otherwise
    bool _is_deriv{false};           // is this the derivative of another signal?

public:
    const std::size_t _index{0};                // the signal index

    ValueSignalInfo(const std::string& full_name, std::size_t index) :
        SignalInfo(full_name), _index(index)
    {
        _value = this;
    }

    bool is_state() const {return _deriv_sig;}
    bool is_deriv() const {return _is_deriv;}
    ValueSignal deriv_info() const {return _deriv_sig;}
};

class ScalarSignalInfo : public ValueSignalInfo
{
    friend class SignalRegistry;

protected:
    double _iv; // the initial value, only valid for states, i.e. if _deriv_sig != nullptr

public:
    ScalarSignalInfo(const std::string& full_name, std::size_t index) : ValueSignalInfo(full_name, index)
    {
        _scalar = this;
    }

    double iv() const {return _iv;}
};

class ArraySignalInfo : public ValueSignalInfo
{
    friend class SignalRegistry;

protected:
    Array _iv; // the initial value, only valid for states, i.e. if _deriv_sig != nullptr

public:
    const std::size_t _size;

    ArraySignalInfo(const std::string& full_name, std::size_t index, std::size_t size) :
        ValueSignalInfo(full_name, index), _size(size)
    {
        _array = this;
    }

    const Array& iv() const {return _iv;}
};

struct BusSignalInfo : public SignalInfo
{
    friend class SignalRegistry;

protected:
    const std::vector<std::pair<std::string, Signal>> _signals;

public:
    BusSignalInfo(const std::string& full_name) : SignalInfo(full_name)
    {
        _bus = this;
    }
};

class SignalRegistry
{
public:
    using SignalInfos = std::vector<Signal>;

protected:
    SignalInfos _signal_infos;

    ValueSignalInfo* _register_state(Signal sig, Signal deriv_sig);

public:
    ~SignalRegistry();

    const SignalInfos& signals() const {return _signal_infos;}

    void register_state(Signal sig, Signal deriv_sig, double iv)
    {
        _register_state(sig, deriv_sig)->_scalar->_iv = iv;
    }

    void register_state(Signal sig, Signal deriv_sig, const Array& iv)
    {
        _register_state(sig, deriv_sig)->_array->_iv = iv;
    }

    Signal           find_signal(const std::string& name, bool exact_match=false) const;
    ScalarSignal register_signal(const std::string& name);
    ArraySignal  register_signal(const std::string& name, std::size_t size);
};

class Signals : public std::vector<Signal>
{
public:
    using _Parent = std::vector<Signal>;
    using _Parent::vector;

    Signals(Signal signal)
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

class Values
{
public:
    struct ValueInfo
    {
        const ValueSignalInfo& _si;              // corresponding signal info
        bool _assigned{false};                   // has the value been assigned?

        double* _scalar{nullptr};                // valid if this is a scalar signal, nullptr otherwise
        Eigen::Map<Eigen::ArrayXd> _array;       // used only if this is an array signal, empty otherwise

        double* _deriv_scalar{nullptr};          // only valid if _is_deriv member of the signal info is true and this is a scalar signal, nullptr otherwise
        Eigen::Map<Eigen::ArrayXd> _deriv_array; // used only if _is_deriv member of the signal info is true and this is an array signal, empty and unused otherwise
    
        ValueInfo(const ValueSignalInfo& si, double* data, std::size_t size) :
            _si(si), _scalar(size == 0 ? data : nullptr), _array(size == 0 ? nullptr : data, size),
            _deriv_array(nullptr, 0) {}

        bool is_assigned() const {return _assigned;}
    };

protected:
    std::vector<ValueInfo> _value_infos;
    std::size_t             _total_size{0};
    Eigen::ArrayXd              _values;
    Eigen::Map<Eigen::ArrayXd>  _states{nullptr, Eigen::Dynamic};
    Eigen::ArrayXd              _derivs;

    inline ValueInfo& get_value_info(ValueSignal si)
    {
        verify(si, "invalid signal info!");
        return _value_infos[si->_index];
    }

    template<typename T>
    auto _get(const ValueInfo& vi) const -> const auto;

    template<typename T>
    void _set(ValueInfo& vi, const T& value)
    {
        verify(vi._array.rows() == value.rows(),
            std::string("size mismatch (id=") + vi._si._full_name + ")(" + std::to_string(vi._array.rows()) +
            " vs " + std::to_string(value.rows()) + ")!");
        vi._array = value;
        if (vi._si.is_deriv())
            vi._deriv_array = value;
        vi._assigned = true;
    }

public:
    Values(const pooya::Model& model);

    inline const ValueInfo& get_value_info(Signal sig) const
    {
        verify(sig, "invalid signal!");
        verify(sig->as_value(), sig->_full_name + ": value signal needed!");
        return _value_infos[sig->as_value()->_index];
    }

    bool valid(Signal sig) const
    {
        return get_value_info(sig).is_assigned();
    }

    const decltype(_value_infos)& value_infos() const {return _value_infos;}
    const decltype(_values)& values() const {return _values;}
    const decltype(_states)& states() const {return _states;}
    const decltype(_derivs)& derivs() const {return _derivs;}

    template<typename T>
    auto get(Signal sig) const -> const auto
    {
        const auto& vi = get_value_info(sig);
        verify(vi.is_assigned(), "attempting to access an unassigned value!");
        return _get<T>(vi);
    }

    double get_scalar(Signal sig) const;
    auto    get_array(Signal sig) const;

    template<typename T>
    void set(Signal sig, const T& value)
    {
        auto& vi = get_value_info(sig->as_value());
        verify(!vi.is_assigned(), sig->_full_name + ": re-assignment is prohibited!");
        _set<T>(vi, value);
    }

    void set_scalar(Signal sig, double value);
    void  set_array(Signal sig, const Array& value);

    void set_states(const Eigen::ArrayXd& states);

    void stream(std::ostream& os) const
    {
        int k = 0;
        for (const auto& vi: _value_infos)
        {
            os << "- [" << k++ << "]: ";
            (vi.is_assigned() ? (vi._scalar ? os << *vi._scalar : os << vi._array) : os << "*") << "\n";
        }
        os << "\n";
    }
};

template<>
inline auto Values::_get<Array>(const Values::ValueInfo& vi) const -> const auto
{
    verify(!vi._scalar, vi._si._full_name + ": attempting to retrieve a scalar as an array!");
    return vi._array;
}

template<>
inline auto Values::_get<double>(const Values::ValueInfo& vi) const -> const auto
{
    verify(vi._scalar, "attempting to retrieve an array as a scalar!");
    return *vi._scalar;
}

inline double Values::get_scalar(Signal sig) const {return get<double>(sig);}
inline auto   Values::get_array (Signal sig) const {return get<Array> (sig);}

template<>
inline void Values::_set<double>(ValueInfo& vi, const double& value)
{
    verify(vi._scalar, "cannot assign scalar to array!");
    *vi._scalar = value;
    if (vi._si.is_deriv())
        *vi._deriv_scalar = value;
    vi._assigned = true;
}

inline void Values::set_scalar(Signal sig,       double value) {set<double>(sig, value);}
inline void Values::set_array (Signal sig, const Array& value) {set<Array> (sig, value);}

inline std::ostream& operator<<(std::ostream& os, const Values& values)
{
    values.stream(os);
    return os;
}

}

#endif // __POOYA_SIGNAL_HPP__
