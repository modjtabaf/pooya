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

class Signal
{
public:
    using Id = std::size_t;
    static constexpr Id NoId = std::string::npos;

protected:
    std::string _given_name;
    std::string  _full_name; // TODO: maybe replace this with a pointer to signal info
    std::size_t         _id{NoId};
    std::size_t       _size{0};

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

class SignalRegistry
{
public:
    struct SignalInfo
    {
        std::string _full_name;               // full name of the signal
        Signal::Id         _id{Signal::NoId}; // the signal ID (must be valid, i.e. != NoId)
        Signal::Id   _deriv_id{Signal::NoId}; // the ID valid of the derivative signal if this is a state variable, NoId otherwise
        bool         _is_deriv{false};        // is this the derivative of another signal?
        std::size_t      _size{0};            // array size if > 0, indicates a scalar if == 0
        Array              _iv;               // the initial value, only valid for states, i.e. if _deriv_id != Signal::NoId

        SignalInfo(const std::string& full_name, Signal::Id id, std::size_t size) : _full_name(full_name), _id(id), _size(size)
        {
            verify(id != Signal::NoId, full_name + ": invalid signal id!");
        }

        bool is_state() const {return _deriv_id != Signal::NoId;}
        bool is_scalar() const {return _size == 0;}
    };

    using SignalInfos = std::vector<SignalInfo>;

protected:
    SignalInfos _signal_infos;

    SignalInfos::const_iterator _find_signal(const std::string& name, bool exact_match = false) const;
    SignalInfo& _register_state(Signal::Id id, Signal::Id deriv_id);

public:
    const SignalInfos& signals() const {return _signal_infos;}
    std::size_t num_signals() const {return _signal_infos.size();} // deprecate

    const SignalInfo& get_signal_by_id(Signal::Id id) const
    {
        return _signal_infos[id];
    }

    void register_state(Signal::Id id, Signal::Id deriv_id, const Array& iv)
    {
        _register_state(id, deriv_id)._iv = iv;
    }

    void register_state(Signal::Id id, Signal::Id deriv_id, double iv)
    {
        _register_state(id, deriv_id)._iv << iv;
    }

    Signal::Id find_signal(const std::string& name, bool exact_match = false) const;
    Signal::Id register_signal(const std::string& name, std::size_t size);
};

class Values
{
public:
    struct ValueInfo
    {
        const SignalRegistry::SignalInfo& _si;   // corresponding signal info
        bool _assigned{false};                   // has the value been assigned?

        double* _scalar{nullptr};                // valid if this is a scalar signal, nullptr otherwise
        Eigen::Map<Eigen::ArrayXd> _array;       // used only if this is an array signal, empty otherwise

        double* _deriv_scalar{nullptr};          // only valid if _is_deriv member of the signal info is true and this is a scalar signal, nullptr otherwise
        Eigen::Map<Eigen::ArrayXd> _deriv_array; // used only if _is_deriv member of the signal info is true and this is an array signal, empty and unused otherwise
    
        ValueInfo(const SignalRegistry::SignalInfo& si, double* data, std::size_t size) :
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

    inline ValueInfo& get_value_info(Signal::Id id)
    {
        verify(id != Signal::NoId, "invalid id!");
        return _value_infos[id];
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

    inline const ValueInfo& get_value_info(Signal::Id id) const
    {
        verify(id != Signal::NoId, "invalid id!");
        return _value_infos[id];
    }

    bool valid(Signal::Id id) const
    {
        return get_value_info(id).is_assigned();
    }

    bool is_array(Signal::Id id) const
    {
        return get_value_info(id)._scalar == nullptr;
    }

    const decltype(_values)& values() const {return _values;}
    const decltype(_states)& states() const {return _states;}
    const decltype(_derivs)& derivs() const {return _derivs;}

    template<typename T>
    auto get(Signal::Id id) const -> const auto
    {
        const auto& vi = get_value_info(id);
        verify(vi.is_assigned(), "attempting to access an unassigned value!");
        return _get<T>(vi);
    }

    double get_scalar(Signal::Id id) const;
    auto   get_array(Signal::Id id) const;

    template<typename T>
    void set(Signal::Id id, const T& value)
    {
        verify(id != Signal::NoId, "invalid id!");
        auto& vi = get_value_info(id);
        verify(!vi.is_assigned(), "re-assignment is prohibited!");
        _set<T>(vi, value);
    }

    void set_scalar(Signal::Id id, double value);
    void set_array(Signal::Id id, const Array& value);

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

inline double Values::get_scalar(Signal::Id id) const {return get<double>(id);}
inline auto   Values::get_array (Signal::Id id) const {return get<Array> (id);}

template<>
inline void Values::_set<double>(ValueInfo& vi, const double& value)
{
    verify(vi._scalar, "cannot assign scalar to array!");
    *vi._scalar = value;
    if (vi._si._is_deriv)
        *vi._deriv_scalar = value;
    vi._assigned = true;
}

inline void Values::set_scalar(Signal::Id id, double value) {set<double>(id, value);}
inline void Values::set_array (Signal::Id id, const Array& value) {set<Array>(id, value);}

inline std::ostream& operator<<(std::ostream& os, const Values& values)
{
    values.stream(os);
    return os;
}

}

#endif // __POOYA_SIGNAL_HPP__
