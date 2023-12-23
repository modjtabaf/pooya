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

struct SignalInfo
{
    std::string _full_name;               // full name of the signal
    std::size_t     _index{0};            // the signal index (must be valid, i.e. > 0)
    const SignalInfo* _deriv_si{nullptr}; // pointer to the derivative signal info if this is a state variable, nullptr otherwise
    bool         _is_deriv{false};        // is this the derivative of another signal?
    std::size_t      _size{0};            // array size if > 0, indicates a scalar if == 0
    Array              _iv;               // the initial value, only valid for states, i.e. if _deriv_id != SigNullId

    SignalInfo(const std::string& full_name, std::size_t index, std::size_t size) : _full_name(full_name), _index(index), _size(size)
    {
    }

    bool is_state() const {return _deriv_si;}
    bool is_scalar() const {return _size == 0;}
};

class SignalRegistry
{
public:
    using SignalInfos = std::vector<SignalInfo*>;

protected:
    SignalInfos _signal_infos;

    SignalInfo* _register_state(const SignalInfo* id, const SignalInfo* deriv_id);

public:
    ~SignalRegistry();

    const SignalInfos& signals() const {return _signal_infos;}

    void register_state(const SignalInfo* si, const SignalInfo* deriv_si, const Array& iv)
    {
        _register_state(si, deriv_si)->_iv = iv;
    }

    void register_state(const SignalInfo* si, const SignalInfo* deriv_si, double iv)
    {
        _register_state(si, deriv_si)->_iv << iv;
    }

    const SignalInfo* find_signal(const std::string& name, bool exact_match = false) const;
    SignalInfo* register_signal(const std::string& name, std::size_t size);
};

class Signal
{
protected:
    std::string _given_name;
    const SignalInfo*   _si{nullptr};

    std::string _make_valid_given_name(const std::string& given_name) const;
    void _set_owner(Parent& owner, std::size_t size);

public:
    Signal(const std::string& given_name, Parent& owner, std::size_t size=0) : _given_name(_make_valid_given_name(given_name))
    {
        _set_owner(owner, size);
    }
    Signal(const char* given_name, Parent& owner, std::size_t size=0) : _given_name(_make_valid_given_name(given_name))
    {
        _set_owner(owner, size);
    }
    Signal(Parent& owner, std::size_t size=0) : _given_name(_make_valid_given_name(""))
    {
        _set_owner(owner, size);
    }
    Signal() = default;
    Signal(const Signal& signal) = default;

    Signal& operator=(const Signal& rhs) = default;

    operator const SignalInfo*() const {return _si;}
    const SignalInfo* info() const {return _si;}
    const std::string& given_name() const {return _given_name;}
};

inline std::ostream& operator<<(std::ostream& os, const Signal& signal)
{
    return os << "signal [" << signal.given_name() << "] = " << (const SignalInfo*)signal << "\n";
}

class Signals : public std::vector<Signal>
{
public:
    using _Parent = std::vector<Signal>;
    using _Parent::vector;

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

class Values
{
public:
    struct ValueInfo
    {
        const SignalInfo& _si;                   // corresponding signal info
        bool _assigned{false};                   // has the value been assigned?

        double* _scalar{nullptr};                // valid if this is a scalar signal, nullptr otherwise
        Eigen::Map<Eigen::ArrayXd> _array;       // used only if this is an array signal, empty otherwise

        double* _deriv_scalar{nullptr};          // only valid if _is_deriv member of the signal info is true and this is a scalar signal, nullptr otherwise
        Eigen::Map<Eigen::ArrayXd> _deriv_array; // used only if _is_deriv member of the signal info is true and this is an array signal, empty and unused otherwise
    
        ValueInfo(const SignalInfo& si, double* data, std::size_t size) :
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

    inline ValueInfo& get_value_info(const SignalInfo* si)
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
        if (vi._si._is_deriv)
            vi._deriv_array = value;
        vi._assigned = true;
    }

public:
    Values(const pooya::Model& model);

    inline const ValueInfo& get_value_info(const SignalInfo* si) const
    {
        verify(si, "invalid signal info!");
        return _value_infos[si->_index];
    }

    bool valid(const SignalInfo* si) const
    {
        return get_value_info(si).is_assigned();
    }

    bool is_array(const SignalInfo* si) const
    {
        return get_value_info(si)._scalar == nullptr;
    }

    const decltype(_value_infos)& value_infos() const {return _value_infos;}
    const decltype(_values)& values() const {return _values;}
    const decltype(_states)& states() const {return _states;}
    const decltype(_derivs)& derivs() const {return _derivs;}

    template<typename T>
    auto get(const SignalInfo* si) const -> const auto
    {
        const auto& vi = get_value_info(si);
        verify(vi.is_assigned(), "attempting to access an unassigned value!");
        return _get<T>(vi);
    }

    double get_scalar(const SignalInfo* si) const;
    auto   get_array(const SignalInfo* si) const;

    template<typename T>
    void set(const SignalInfo* si, const T& value)
    {
        verify(si, "invalid signal info!");
        auto& vi = get_value_info(si);
        verify(!vi.is_assigned(), "re-assignment is prohibited!");
        _set<T>(vi, value);
    }

    void set_scalar(const SignalInfo* si, double value);
    void set_array(const SignalInfo* si, const Array& value);

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

inline double Values::get_scalar(const SignalInfo* si) const {return get<double>(si);}
inline auto   Values::get_array (const SignalInfo* si) const {return get<Array> (si);}

template<>
inline void Values::_set<double>(ValueInfo& vi, const double& value)
{
    verify(vi._scalar, "cannot assign scalar to array!");
    *vi._scalar = value;
    if (vi._si._is_deriv)
        *vi._deriv_scalar = value;
    vi._assigned = true;
}

inline void Values::set_scalar(const SignalInfo* si, double value) {set<double>(si, value);}
inline void Values::set_array (const SignalInfo* si, const Array& value) {set<Array>(si, value);}

inline std::ostream& operator<<(std::ostream& os, const Values& values)
{
    values.stream(os);
    return os;
}

}

#endif // __POOYA_SIGNAL_HPP__
