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

#include <initializer_list>
#include <map>
#include <string>

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
        pooya_trace0;
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

class        SignalInfo;
class   ValueSignalInfo;
class  ScalarSignalInfo;
class IntegerSignalInfo;
class   ArraySignalInfo;
class     BusSignalInfo;

using        Signal = const        SignalInfo*;
using   ValueSignal = const   ValueSignalInfo*;
using  ScalarSignal = const  ScalarSignalInfo*;
using IntegerSignal = const IntegerSignalInfo*;
using   ArraySignal = const   ArraySignalInfo*;
using     BusSignal = const     BusSignalInfo*;

using LabelSignal = std::pair<std::string, Signal>;
using LabelSignalList = std::vector<LabelSignal>;
using LabelSignalMap = std::map<LabelSignal::first_type, LabelSignal::second_type>;

template<typename T>
struct Types
{
    using Signal = ArraySignal;
    using GetValue = const Eigen::Map<Eigen::ArrayXd>&;
    using SetValue = const Array&;
};

template<>
struct Types<double>
{
    using Signal = ScalarSignal;
    using GetValue = double;
    using SetValue = double;
};

template<>
struct Types<int>
{
    using Signal = IntegerSignal;
    using GetValue = int;
    using SetValue = int;
};

class SignalInfo
{
    friend class Model;

public:
    const std::string _full_name; // full name of the signal
    const std::size_t _index{0};  // the signal index

protected:

    ValueSignalInfo*     _value{nullptr};
    ScalarSignalInfo*   _scalar{nullptr};
    ArraySignalInfo*     _array{nullptr};
    IntegerSignalInfo* _integer{nullptr};
    BusSignalInfo*         _bus{nullptr};

    SignalInfo(const std::string& full_name, std::size_t index) : _full_name(full_name), _index(index) {}

public:
    ValueSignal     as_value() const {return   _value;}
    ScalarSignal   as_scalar() const {return  _scalar;}
    IntegerSignal as_integer() const {return _integer;}
    ArraySignal     as_array() const {return   _array;}
    BusSignal         as_bus() const {return     _bus;}
};

class LabelSignals
{
protected:
    LabelSignalMap _label_signals_map;
    LabelSignalList _label_signals_list;

    std::string _make_auto_label(std::size_t index) const {return "sig" + std::to_string(index);}

    void _init(LabelSignalList::const_iterator begin_, LabelSignalList::const_iterator end_);

public:
    LabelSignals() = default;
    LabelSignals(Signal signal);
    LabelSignals(const std::initializer_list<Signal>& il);
    LabelSignals(const std::initializer_list<LabelSignal>& il);

    // explicit binding by label
    void bind(const std::string& label, ScalarSignal& sig) const
    {
        pooya_trace("label: " + label);
        verify(_label_signals_map.find(label) != _label_signals_map.end(), "no signal labeled " + label + "!");
        verify_scalar_signal(_label_signals_map.at(label));
        sig = _label_signals_map.at(label)->as_scalar();
    }

    void bind(const std::string& label, IntegerSignal& sig) const
    {
        pooya_trace("label: " + label);
        verify(_label_signals_map.find(label) != _label_signals_map.end(), "no signal labeled " + label + "!");
        verify_integer_signal(_label_signals_map.at(label));
        sig = _label_signals_map.at(label)->as_integer();
    }

    void bind(const std::string& label, ArraySignal& sig) const
    {
        pooya_trace("label: " + label);
        verify(_label_signals_map.find(label) != _label_signals_map.end(), "no signal labeled " + label + "!");
        verify_array_signal(_label_signals_map.at(label));
        sig = _label_signals_map.at(label)->as_array();
    }

    void bind(const std::string& label, BusSignal& sig) const
    {
        pooya_trace("label: " + label);
        verify(_label_signals_map.find(label) != _label_signals_map.end(), "no signal labeled " + label + "!");
        verify_bus_signal(_label_signals_map.at(label));
        sig = _label_signals_map.at(label)->as_bus();
    }

    // explicit binding by index
    void bind(std::size_t index, ScalarSignal& sig) const
    {
        pooya_trace("index: " + std::to_string(index));
        verify_scalar_signal(_label_signals_list.at(index).second);
        sig = _label_signals_list.at(index).second->as_scalar();
    }

    void bind(std::size_t index, IntegerSignal& sig) const
    {
        pooya_trace("index: " + std::to_string(index));
        verify_integer_signal(_label_signals_list.at(index).second);
        sig = _label_signals_list.at(index).second->as_integer();
    }

    void bind(std::size_t index, ArraySignal& sig) const
    {
        pooya_trace("index: " + std::to_string(index));
        verify_array_signal(_label_signals_list.at(index).second);
        sig = _label_signals_list.at(index).second->as_array();
    }

    void bind(std::size_t index, BusSignal& sig) const
    {
        pooya_trace("index: " + std::to_string(index));
        verify_bus_signal(_label_signals_list.at(index).second);
        sig = _label_signals_list.at(index).second->as_bus();
    }

    // implicit binding by index
    template<typename SignalType>
    void bind(SignalType& sig) const
    {
        pooya_trace0;
        verify(_label_signals_list.size() == 1, "Implicit binding is allowed only if there is one and only one signal.");
        bind(0, sig);
    }

    using const_iterator = LabelSignalList::const_iterator;

    Signal operator[](std::size_t index) const {return _label_signals_list[index].second;}
    Signal operator[](const std::string& label) const {return _label_signals_map.at(label);}
    void clear() noexcept
    {
        _label_signals_map.clear();
        _label_signals_list.clear();
    }
    std::size_t size() const noexcept
    {
        return _label_signals_list.size();
    }
    const_iterator begin() const noexcept {return _label_signals_list.begin();}
    const_iterator end() const noexcept {return _label_signals_list.end();}
    void reserve(std::size_t new_cap)
    {
        _label_signals_list.reserve(new_cap);
    }
    bool push_back(const LabelSignal& ls)
    {
        pooya_trace0;
        verify(!ls.first.empty(), "Signal label can not be empty!");
        verify(ls.second, ls.first + ": invalid signal!");
        if (ls.first.empty() || (ls.second == nullptr))
            return false;

        auto it = _label_signals_map.find(ls.first);
        verify(it == _label_signals_map.end(), ls.first + ": label already exists!");
        if (it != _label_signals_map.end())
            return false;

        _label_signals_map[ls.first] = ls.second;
        _label_signals_list.push_back(ls);
        return true;
    }
    void shrink_to_fit() {_label_signals_list.shrink_to_fit();}
};

class ValueSignalInfo : public SignalInfo
{
    friend class Model;

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
    friend class Model;

protected:
    double _iv; // the initial value, only valid for states, i.e. if _deriv_sig != nullptr

    ScalarSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) : ValueSignalInfo(full_name, index, vi_index)
    {
        _scalar = this;
    }

public:
    double iv() const {return _iv;}
};

class IntegerSignalInfo : public ValueSignalInfo
{
    friend class Model;

protected:
    IntegerSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) : ValueSignalInfo(full_name, index, vi_index)
    {
        _integer = this;
    }
};

class ArraySignalInfo : public ValueSignalInfo
{
    friend class Model;

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

class BusSpec
{
    friend class BusSignalInfo;

public:
    struct WireInfo
    {
        const std::string _label;
        const BusSpec* _bus{nullptr};
        const bool _scalar{false};
        const std::size_t _array_size{0};

        // scalar
        WireInfo(const std::string& label) : _label(label), _scalar(true) {}

        // array
        WireInfo(const std::string& label, std::size_t array_size) : _label(label), _array_size(array_size)
        {
            verify(array_size > 0, "array size cannot be zero!");
        }

        // bus
        WireInfo(const std::string& label, const BusSpec& bus) : _label(label), _bus(&bus) {}
    };

    const std::vector<WireInfo> _wires;

    // empty
    BusSpec() = default;

    template<typename Iter>
    BusSpec(Iter begin_, Iter end_) : _wires(begin_, end_) {}

    BusSpec(const std::initializer_list<WireInfo>& l) : _wires(l) {}

    std::size_t total_size() const
    {
        pooya_trace0;
        std::size_t ret = _wires.size();
        for (const auto& wi: _wires)
            if (wi._bus)
                ret += wi._bus->total_size();
        return ret;
    }

    std::size_t index_of(const std::string& label) const
    {
        pooya_trace("label: " + label);
        return std::distance(_wires.begin(),
            std::find_if(_wires.begin(), _wires.end(),
                [&](const WireInfo& wi)
                {
                    return wi._label == label;
                }
            ));
    }

    bool operator==(const BusSpec& other) const {return this == &other;}
};

struct BusSignalInfo : public SignalInfo
{
    friend class Model;

public:
    const BusSpec& _spec;

protected:
    LabelSignalList _signals;

protected:
    void _set(std::size_t index, Signal sig);

    BusSignalInfo(const std::string& full_name, std::size_t index, const BusSpec& spec, LabelSignalList::const_iterator begin_, LabelSignalList::const_iterator end_) :
        SignalInfo(full_name, index), _spec(spec)
    {
        pooya_trace("fullname: " + full_name);
        verify(std::size_t(std::distance(begin_, end_)) == _spec._wires.size(), "incorrect number of signals: " + std::to_string(std::size_t(std::distance(begin_, end_))));
        _signals.reserve(_spec._wires.size());
        for(const auto& wi: _spec._wires)
            _signals.push_back({wi._label, nullptr});
        for (auto& it = begin_; it != end_; it++)
            _set(_spec.index_of(it->first), it->second);
#if !defined(NDEBUG)
        for (const auto& ls: _signals)
        {
            verify(ls.second, "Unassigned wire detected: " + ls.first);
        }
#endif // !defined(NDEBUG)
        _bus = this;
    }

public:
    const BusSpec& spec() const {return _spec;}
    std::size_t size() const {return _signals.size();}

    const LabelSignal& operator[](std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        verify(index < _signals.size(), "index out of range!");
        return _signals[index];
    }

    const LabelSignal& at(std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        return _signals.at(index);
    }

    Signal operator[](const std::string& label) const;
    Signal at(const std::string& label) const;

    ScalarSignal scalar_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        Signal sig = at(label);
        verify_scalar_signal(sig);
        return sig->as_scalar();
    }

    IntegerSignal integer_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        Signal sig = at(label);
        verify_integer_signal(sig);
        return sig->as_integer();
    }

    ArraySignal array_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        Signal sig = at(label);
        verify_array_signal(sig);
        return sig->as_array();
    }

    BusSignal bus_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        Signal sig = at(label);
        verify_bus_signal(sig);
        return sig->as_bus();
    }
};

std::ostream& operator<<(std::ostream& os, const LabelSignals& signals);

class Value
{
protected:
    const bool _is_scalar{true};
    double _scalar{0.0};
    Array _array;

public:
    Value(double v) : _scalar(v) {}
    Value(const Array& v) : _is_scalar(false), _array(v) {}

    bool is_scalar() const {return _is_scalar;}

    double as_scalar() const
    {
        pooya_trace0;
        verify(_is_scalar, "attempting to retrieve an array as a scalar!");
        return _scalar;
    }

    const Array& as_array() const
    {
        pooya_trace0;
        verify(!_is_scalar, "attempting to retrieve a scalar as an array!");
        return _array;
    }
};

struct ValueInfo
{
    friend class Values;

public:
    const ValueSignalInfo& _si;                          // corresponding signal info

protected:
    bool _assigned{false};                               // has the value been assigned?

    double* _scalar{nullptr};                            // valid if this is a scalar signal (either integer or float), nullptr otherwise
    bool _integer{false};                                // used only if this is a scalar signal, false otherwise
    Eigen::Map<Eigen::ArrayXd> _array{nullptr, 0};       // used only if this is an array signal, empty otherwise

    double* _deriv_scalar{nullptr};                      // only valid if _is_deriv member of the signal info is true and this is a float scalar signal, nullptr otherwise
    Eigen::Map<Eigen::ArrayXd> _deriv_array{nullptr, 0}; // used only if _is_deriv member of the signal info is true and this is an array signal, empty and unused otherwise

    // float scalar or array
    ValueInfo(const ValueSignalInfo& si, double* data, std::size_t float_size) :
        _si(si), _scalar(float_size == 0 ? data : nullptr), _array(float_size == 0 ? nullptr : data, float_size) {}

    // integer scalar
    ValueInfo(const ValueSignalInfo& si, double* data) :
        _si(si), _scalar(data), _integer(true) {}

public:
    template<typename T>
    typename Types<T>::GetValue get() const
    {
        pooya_trace0;
        verify(_scalar == nullptr, _si._full_name + ": attempting to retrieve a scalar as an array!");
        verify(_assigned, _si._full_name + ": attempting to access an unassigned value!");
        return _array;
    }

    template<typename T>
    void set(typename Types<T>::SetValue value)
    {
        pooya_trace0;
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
    bool is_integer() const {return _integer && is_scalar();}
    std::size_t size() const
    {
        return _scalar == nullptr ? _array.size() : 0;
    }
};

template<>
inline typename Types<double>::GetValue ValueInfo::get<double>() const
{
    pooya_trace0;
    verify(is_scalar(), _si._full_name + ": attempting to retrieve an array as a scalar!");
    verify(is_assigned(), _si._full_name + ": attempting to access an unassigned value!");
    return *_scalar;
}

template<>
inline typename Types<int>::GetValue ValueInfo::get<int>() const
{
    pooya_trace0;
    verify(is_scalar(), _si._full_name + ": attempting to retrieve an array as an integer!");
    verify(is_integer(), _si._full_name + ": attempting to retrieve a float as an integer!");
    verify(is_assigned(), _si._full_name + ": attempting to access an unassigned value!");
    return std::round(*_scalar);
}

template<>
inline void ValueInfo::set<int>(typename Types<int>::SetValue value)
{
    pooya_trace("value: " + std::to_string(value));
    verify(is_scalar(), _si._full_name + ": cannot assign an integer to a non-scalar!");
    verify(is_integer(), _si._full_name + ": cannot assign an integer to a float!");
    verify(!is_assigned(), _si._full_name + ": re-assignment is prohibited!");
    *_scalar = value;
    _assigned = true;
}

template<>
inline void ValueInfo::set<double>(typename Types<double>::SetValue value)
{
    pooya_trace("value: " + std::to_string(value));
    if (is_integer())
    {
        set<int>(std::round(value));
        return;
    }

    verify(is_scalar(), _si._full_name + ": cannot assign a scalar to a non-scalar!");
    verify(!is_assigned(), _si._full_name + ": re-assignment is prohibited!");
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
    Eigen::Map<Eigen::ArrayXd>  _states{nullptr, 0};
    Eigen::ArrayXd              _derivs;

    inline ValueInfo& get_value_info(Signal sig)
    {
        pooya_trace0;
        verify_value_signal(sig);
        return _value_infos[sig->as_value()->_vi_index];
    }

public:
    Values(const pooya::Model& model);

    inline const ValueInfo& get_value_info(Signal sig) const
    {
        pooya_trace0;
        verify(sig, "invalid signal!");
        verify(sig->as_value(), sig->_full_name + ": value signal needed!");
        return _value_infos[sig->as_value()->_vi_index];
    }

    bool valid(Signal sig) const
    {
        pooya_trace0;
        return get_value_info(sig).is_assigned();
    }

    const decltype(_value_infos)& value_infos() const {return _value_infos;}
    const decltype(_values)& values() const {return _values;}
    const decltype(_states)& states() const {return _states;}
    const decltype(_derivs)& derivs() const {return _derivs;}

    template<typename T>
    typename Types<T>::GetValue get(Signal sig) const
    {
        pooya_trace0;
        return get_value_info(sig).get<T>();
    }

    typename Types<double>::GetValue get(ScalarSignal  sig) const {return get<double>(sig);}
    typename Types<int   >::GetValue get(IntegerSignal sig) const {return get<int   >(sig);}
    typename Types<Array >::GetValue get(ArraySignal   sig) const {return get<Array >(sig);}

    template<typename T>
    void set(Signal sig, typename Types<T>::SetValue value)
    {
        pooya_trace0;
        get_value_info(sig).set<T>(value);
    }

    void set(ScalarSignal sig, double value) {set<double>(sig, value);}
    void set(IntegerSignal sig, double value) {set<int>(sig, std::round(value));} // avoid the default implicit double-to-int conversion
    void set(ArraySignal sig, const Array& value) {set<Array>(sig, value);}

    void set_states(const Eigen::ArrayXd& states);

    void stream(std::ostream& os) const
    {
        pooya_trace0;
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
    pooya_trace0;
    values.stream(os);
    return os;
}

}

#endif // __POOYA_SIGNAL_HPP__
