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
class StatesInfo;

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

class Values
{
public:
    struct ValueInfo
    {
        std::size_t _id;
        bool _assigned{false};

        double* _scalar{nullptr};
        Eigen::Map<Eigen::ArrayXd> _array;

        bool _is_state;

        bool _is_deriv{false};
        double* _deriv_scalar{nullptr};
        Eigen::Map<Eigen::ArrayXd> _deriv_array;
    
        ValueInfo(Signal::Id id, double* data, std::size_t size, bool is_state) :
            _id(id), _scalar(size == 0 ? data : nullptr), _array(size == 0 ? nullptr : data, size),
            _is_state(is_state), _deriv_array(nullptr, 0) {}
    };

protected:
    const SignalRegistry& _sig_reg;
    std::vector<ValueInfo> _value_infos;
    std::size_t _total_size{0};
    Eigen::ArrayXd _values;
    Eigen::Map<Eigen::ArrayXd> _states{nullptr, Eigen::Dynamic};
    Eigen::ArrayXd _derivs;
    std::size_t _states_size{0};

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
            std::string("size mismatch (id=") + std::to_string(vi._id) + ")(" + std::to_string(vi._array.rows()) +
            " vs " + std::to_string(value.rows()) + ")!");
        vi._array = value;
        if (vi._is_deriv)
            vi._deriv_array = value;
    }

public:
    Values(const pooya::Model& model);
    Values(const pooya::Model& model, const StatesInfo& states);

    inline const ValueInfo& get_value_info(Signal::Id id) const
    {
        verify(id != Signal::NoId, "invalid id!");
        return _value_infos[id];
    }

    bool valid(Signal::Id id) const
    {
        return get_value_info(id)._assigned;
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
        verify(vi._assigned, "attempting to access an unassigned value!");
        return _get<T>(vi);
    }

    double get_scalar(Signal::Id id) const;
    auto   get_array(Signal::Id id) const;

    template<typename T>
    void set(Signal::Id id, const T& value)
    {
        verify(id != Signal::NoId, "invalid id!");
        auto& vi = get_value_info(id);
        verify(!vi._assigned, "re-assignment is prohibited!");
        _set<T>(vi, value);
        vi._assigned = true;
    }

    void set_scalar(Signal::Id id, double value);
    void set_array(Signal::Id id, const Array& value);

    void set_states(const Eigen::ArrayXd& states);

    void invalidate()
    {
        for (auto& vi: _value_infos) vi._assigned = false;
    }

    void stream(std::ostream& os) const
    {
        int k = 0;
        for (const auto& vi: _value_infos)
        {
            os << "- [" << k++ << "]: ";
            (vi._assigned ? os << vi._array : os << "*") << "\n";
        }
        os << "\n";
    }
};

template<>
inline auto Values::_get<Array>(const Values::ValueInfo& vi) const -> const auto
{
    verify(!vi._scalar, _sig_reg.get_signal_by_id(vi._id).first + ": attempting to retrieve a scalar as an array!");
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
    if (vi._is_deriv)
        *vi._deriv_scalar = value;
}

inline void Values::set_scalar(Signal::Id id, double value) {set<double>(id, value);}
inline void Values::set_array (Signal::Id id, const Array& value) {set<Array>(id, value);}

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
    Array _value;
    StateInfo(Signal::Id id, Signal::Id deriv_id, const Array& iv) :
        _id(id), _deriv_id(deriv_id), _scalar(false), _value(iv)
        {
        }
    StateInfo(Signal::Id id, Signal::Id deriv_id, double iv) :
        _id(id), _deriv_id(deriv_id), _scalar(true), _value(iv)
        {
        }
};

class StatesInfo : protected std::vector<StateInfo>
{
protected:
    using _Parent = std::vector<StateInfo>;

    bool _locked{false};
    Array _value;

public:
    using _Parent::vector;

    template<typename T=double>
    void add(Signal::Id state, const T& value, Signal::Id deriv)
    {
        verify(!_locked, "adding to a locked StatesInfo object is prohibited!");
        verify(std::find_if(begin(), end(),
            [&] (const StateInfo& info) -> bool
            {
                return info._id == state;
            }) == end(), std::string("a state with id ") + std::to_string(state) + " is added already!");
        emplace_back(StateInfo(state, deriv, value));
    }

    void lock();

    auto begin() const -> const auto {return _Parent::begin();}
    auto end() const -> const auto {return _Parent::end();}
    auto size() const -> const auto {return _Parent::size();}
    const Array& value() const
    {
        verify(_locked, "getting the value of an unlocked StatesInfo object is prohibited!");
        return _value;
    }

    void set_value(const Array& value)
    {
        verify(_locked, "setting the value of an unlocked StatesInfo object is prohibited!");
        _value = value;
    }
};

}

#endif // __POOYA_SIGNAL_HPP__
