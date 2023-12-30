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

using Array  = ArrayN<>;
using Array1 = ArrayN<1>; // use a scalar instead
using Array2 = ArrayN<2>;
using Array3 = ArrayN<3>;
using Array4 = ArrayN<4>;
using Array5 = ArrayN<5>;
using Array6 = ArrayN<6>;
using Array7 = ArrayN<7>;
using Array8 = ArrayN<8>;
using Array9 = ArrayN<9>;

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

template<typename T>
struct Types
{
    using Signal = ArraySignal;
};

template<>
struct Types<double>
{
    using Signal = ScalarSignal;
};

class SignalInfo
{
    friend class SignalRegistry;

public:
    const std::string _full_name; // full name of the signal
    const std::size_t _index{0};  // the signal index

protected:

    ValueSignalInfo*   _value{nullptr};
    ScalarSignalInfo* _scalar{nullptr};
    ArraySignalInfo*   _array{nullptr};
    BusSignalInfo*       _bus{nullptr};

    SignalInfo(const std::string& full_name, std::size_t index) : _full_name(full_name), _index(index) {}

public:
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

    ValueSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) :
        SignalInfo(full_name, index), _vi_index(vi_index)
    {
        _value = this;
    }

public:
    const std::size_t _vi_index{0};   // the index of associated ValueInfo

    bool is_state() const {return _deriv_sig;}
    bool is_deriv() const {return _is_deriv;}
    ValueSignal deriv_info() const {return _deriv_sig;}
};

class ScalarSignalInfo : public ValueSignalInfo
{
    friend class SignalRegistry;

protected:
    double _iv; // the initial value, only valid for states, i.e. if _deriv_sig != nullptr

    ScalarSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) : ValueSignalInfo(full_name, index, vi_index)
    {
        _scalar = this;
    }

public:
    double iv() const {return _iv;}
};

class ArraySignalInfo : public ValueSignalInfo
{
    friend class SignalRegistry;

protected:
    Array _iv; // the initial value, only valid for states, i.e. if _deriv_sig != nullptr

    ArraySignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index, std::size_t size) :
        ValueSignalInfo(full_name, index, vi_index), _size(size)
    {
        _array = this;
    }

public:
    const std::size_t _size;

    const Array& iv() const {return _iv;}
};

struct BusSignalInfo : public SignalInfo
{
    friend class SignalRegistry;

public:
    using NameSignal = std::pair<std::string, Signal>;

protected:
    const std::vector<NameSignal> _signals;

    BusSignalInfo(const std::string& full_name, std::size_t index, std::initializer_list<NameSignal> l) :
        SignalInfo(full_name, index), _signals(l)
    {
#if !defined(NDEBUG)
        for (const auto& ns: _signals)
            verify_valid_signal(ns.second);
#endif // !defined(NDEBUG)
        _bus = this;
    }

public:
    Signal at(const std::string& name) const
    {
        auto it = std::find_if(_signals.begin(), _signals.end(),
            [&](const NameSignal& ns)
            {
                return ns.first == name;
            });
        return it == _signals.end() ? nullptr : it->second;
    }

    Signal at(std::size_t index) const {return _signals[index].second;}
};

class SignalRegistry
{
public:
    using SignalInfos = std::vector<Signal>;

protected:
    SignalInfos _signal_infos;
    std::size_t _vi_index{0};

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
    BusSignal    register_signal(const std::string& name, std::initializer_list<BusSignalInfo::NameSignal> l);
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

    void bind(std::size_t index, ScalarSignal& sig) const
    {
        verify_scalar_signal(at(index));
        sig = at(index)->as_scalar();
    }

    void bind(std::size_t index, ArraySignal& sig) const
    {
        verify_array_signal(at(index));
        sig = at(index)->as_array();
    }

    void bind(std::size_t index, BusSignal& sig) const
    {
        verify_bus_signal(at(index));
        sig = at(index)->as_bus();
    }
};

inline std::ostream& operator<<(std::ostream& os, const Signals& signals)
{
    os << "signals:\n";
    for (const auto& signal: signals)
        os << "- " << signal;
    return os;
}

struct ValueInfo
{
    friend class Values;

public:
    const ValueSignalInfo& _si;              // corresponding signal info

protected:
    bool _assigned{false};                   // has the value been assigned?

    double* _scalar{nullptr};                // valid if this is a scalar signal, nullptr otherwise
    Eigen::Map<Eigen::ArrayXd> _array;       // used only if this is an array signal, empty otherwise

    double* _deriv_scalar{nullptr};          // only valid if _is_deriv member of the signal info is true and this is a scalar signal, nullptr otherwise
    Eigen::Map<Eigen::ArrayXd> _deriv_array; // used only if _is_deriv member of the signal info is true and this is an array signal, empty and unused otherwise

    ValueInfo(const ValueSignalInfo& si, double* data, std::size_t size) :
        _si(si), _scalar(size == 0 ? data : nullptr), _array(size == 0 ? nullptr : data, size),
        _deriv_array(nullptr, 0) {}

public:
    template<typename T>
    const auto& get() const
    {
        verify(_scalar == nullptr, _si._full_name + ": attempting to retrieve a scalar as an array!");
        verify(_assigned, _si._full_name + ": attempting to access an unassigned value!");
        return _array;
    }

    template<typename T>
    void set(const T& value)
    {
        verify(_scalar == nullptr, _si._full_name + ": cannot assign non-scalar to scalar!");
        verify(!_assigned, _si._full_name + ": re-assignment is prohibited!");
        verify(_array.rows() == value.rows(),
            std::string("size mismatch (id=") + _si._full_name + ")(" + std::to_string(_array.rows()) +
            " vs " + std::to_string(value.rows()) + ")!");
        _array = value;
        if (_si.is_deriv())
            _deriv_array = value;
        _assigned = true;
    }

    bool is_assigned() const {return _assigned;}
    bool is_scalar() const {return _scalar != nullptr;}
    std::size_t size() const
    {
        return _scalar == nullptr ? _array.size() : 0;
    }
};

template<>
inline const auto& ValueInfo::get<double>() const
{
    verify(_scalar != nullptr, _si._full_name + ": attempting to retrieve an array as a scalar!");
    verify(_assigned, _si._full_name + ": attempting to access an unassigned value!");
    return *_scalar;
}

template<>
inline void ValueInfo::set<double>(const double& value)
{
    verify(_scalar != nullptr, _si._full_name + ": cannot assign scalar to non-scalar!");
    verify(!_assigned, _si._full_name + ": re-assignment is prohibited!");
    *_scalar = value;
    if (_si.is_deriv())
        *_deriv_scalar = value;
    _assigned = true;
}

class Values
{
protected:
    std::vector<ValueInfo> _value_infos;
    std::size_t             _total_size{0};
    Eigen::ArrayXd              _values;
    Eigen::Map<Eigen::ArrayXd>  _states{nullptr, Eigen::Dynamic};
    Eigen::ArrayXd              _derivs;

    inline ValueInfo& get_value_info(Signal sig)
    {
        verify_value_signal(sig);
        return _value_infos[sig->as_value()->_vi_index];
    }

public:
    Values(const pooya::Model& model);

    inline const ValueInfo& get_value_info(Signal sig) const
    {
        verify(sig, "invalid signal!");
        verify(sig->as_value(), sig->_full_name + ": value signal needed!");
        return _value_infos[sig->as_value()->_vi_index];
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
    const auto& get(Signal sig) const
    {
        return get_value_info(sig).get<T>();
    }

    double      get(ScalarSignal sig) const {return get<double>(sig);}
    const auto& get(ArraySignal  sig) const {return get<Array>(sig);}

    template<typename T>
    void set(Signal sig, const T& value)
    {
        get_value_info(sig).set<T>(value);
    }

    void set(ScalarSignal sig, double value) {set<double>(sig, value);}
    // void set(ArraySignal sig, const Array& value) {set<Array>(sig, value);} // ambiguous

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

inline std::ostream& operator<<(std::ostream& os, const Values& values)
{
    values.stream(os);
    return os;
}

}

#endif // __POOYA_SIGNAL_HPP__
