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

#include <cstddef>
#include <initializer_list>
#include <map>
#include <string>

#include "Eigen/Core"
#include "util.hpp"

namespace pooya
{

class Model;
class Parent;

template<int N> using ArrayN = Eigen::Array<double, N, 1>;

using Array  = ArrayN<Eigen::Dynamic>;
using Array1 = ArrayN<1>; // use a scalar instead
using Array2 = ArrayN<2>;
using Array3 = ArrayN<3>;
using Array4 = ArrayN<4>;
using Array5 = ArrayN<5>;
using Array6 = ArrayN<6>;
using Array7 = ArrayN<7>;
using Array8 = ArrayN<8>;
using Array9 = ArrayN<9>;

using MappedArray = Eigen::Map<Array>;

class       SignalInfo;
class  ValueSignalInfo;
class  FloatSignalInfo;
class ScalarSignalInfo;
class    IntSignalInfo;
class   BoolSignalInfo;
class  ArraySignalInfo;
class          BusInfo;

using       SignalId =       SignalInfo*;
using  ValueSignalId =  ValueSignalInfo*;
using  FloatSignalId =  FloatSignalInfo*;
using ScalarSignalId = ScalarSignalInfo*;
using    IntSignalId =    IntSignalInfo*;
using   BoolSignalId =   BoolSignalInfo*;
using  ArraySignalId =  ArraySignalInfo*;
using          BusId =          BusInfo*;

using LabelSignalId = std::pair<std::string, SignalId>;
using LabelSignalIdList = std::vector<LabelSignalId>;
using LabelSignalIdMap = std::map<LabelSignalId::first_type, LabelSignalId::second_type>;

class SignalInfo
{
    friend class Model;

public:
    const std::string _full_name; // full name of the signal
    const std::size_t _index{0};  // the signal index

protected:
    ValueSignalInfo*   _value{nullptr};
    FloatSignalInfo*   _float{nullptr};
    ScalarSignalInfo* _scalar{nullptr};
    ArraySignalInfo*   _array{nullptr};
    IntSignalInfo*      _int {nullptr};
    BoolSignalInfo*     _bool{nullptr};
    BusInfo*             _bus{nullptr};

    SignalInfo(const std::string& full_name, std::size_t index) : _full_name(full_name), _index(index) {}

public:
    ValueSignalId   as_value() const {return  _value;}
    FloatSignalId   as_float() const {return  _float;}
    ScalarSignalId as_scalar() const {return _scalar;}
    IntSignalId       as_int() const {return    _int;}
    BoolSignalId     as_bool() const {return   _bool;}
    ArraySignalId   as_array() const {return  _array;}
    BusId             as_bus() const {return    _bus;}
};

class LabelSignals
{
protected:
    LabelSignalIdMap _label_signals_map;
    LabelSignalIdList _label_signals_list;

    std::string _make_auto_label(std::size_t index) const {return "sig" + std::to_string(index);}

    void _init(LabelSignalIdList::const_iterator begin_, LabelSignalIdList::const_iterator end_);

public:
    LabelSignals() = default;
    LabelSignals(SignalId signal);
    LabelSignals(const std::initializer_list<SignalId>& il);
    LabelSignals(const std::initializer_list<LabelSignalId>& il);

    using const_iterator = LabelSignalIdList::const_iterator;

    SignalId operator[](std::size_t index) const {return _label_signals_list[index].second;}
    SignalId operator[](const std::string& label) const {return _label_signals_map.at(label);}
    std::size_t size() const noexcept
    {
        return _label_signals_list.size();
    }
    const_iterator begin() const noexcept {return _label_signals_list.begin();}
    const_iterator end() const noexcept {return _label_signals_list.end();}
    bool push_back(const LabelSignalId& ls)
    {
        pooya_trace0;
        pooya_verify(!ls.first.empty(), "SignalId label can not be empty!");
        pooya_verify(ls.second, ls.first + ": invalid signal!");
        if (ls.first.empty() || (ls.second == nullptr))
            return false;

        auto it = _label_signals_map.find(ls.first);
        pooya_verify(it == _label_signals_map.end(), ls.first + ": label already exists!");
        if (it != _label_signals_map.end())
            return false;

        _label_signals_map[ls.first] = ls.second;
        _label_signals_list.push_back(ls);
        return true;
    }
};

class ValueSignalInfo : public SignalInfo
{
    friend class Model;

protected:
    bool _assigned{false};             // has the value been assigned?

    ValueSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) :
        SignalInfo(full_name, index), _vi_index(vi_index)
    {
        _value = this;
    }

public:
    const std::size_t _vi_index{0};   // the index of associated ValueInfo (REMOVE)

    bool is_assigned() const {return _assigned;}
};

class FloatSignalInfo : public ValueSignalInfo
{
    friend class Model;

protected:
    FloatSignalId _deriv_sig{nullptr}; // the derivative signal if this is a state variable, nullptr otherwise
    bool _is_deriv{false};             // is this the derivative of another signal?

    FloatSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) :
        ValueSignalInfo(full_name, index, vi_index)
    {
        _float = this;
    }

public:
    bool is_state_variable() const {return _deriv_sig;}
    // bool is_deriv() const {return _is_deriv;}
    // ValueSignalId deriv_info() const {return _deriv_sig;}
};

class ScalarSignalInfo : public FloatSignalInfo
{
    friend class Model;

protected:
    ScalarSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) : FloatSignalInfo(full_name, index, vi_index)
    {
        _scalar = this;
    }

    double* _scalar_value{nullptr};
    double* _deriv_scalar_value{nullptr};    // only valid if _is_deriv is true, nullptr otherwise
    // ScalarSignalId _deriv_sig{nullptr}; // the derivative signal if this is a state variable, nullptr otherwise

    // bool is_state_variable() const {return _deriv_sig;}
    // bool is_deriv() const {return _deriv_scalar_value;}

    // ScalarSignalId deriv_info() const {return _deriv_sig;}

public:
    double get() const
    {
        pooya_trace0;
        pooya_verify(_scalar_value, _full_name + ": attempting to retrieve the value of an uninitialized scalar signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return *_scalar_value;
    }

    void set(double value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(_scalar_value, _full_name + ": attempting to assign the value of an uninitialized scalar signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        *_scalar_value = value;
        if (_deriv_scalar_value)
        {
            *_deriv_scalar_value = value;
        }
        _assigned = true;
    }

    operator double() const {return get();}
};

class IntSignalInfo : public ValueSignalInfo
{
    friend class Model;

protected:
    IntSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) : ValueSignalInfo(full_name, index, vi_index)
    {
        _int = this;
    }

    double* _int_value{nullptr};

public:
    int get() const
    {
        pooya_trace0;
        pooya_verify(_int_value, _full_name + ": attempting to retrieve the value of an uninitialized int signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return std::round(*_int_value);
    }

    void set(double value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(_int_value, _full_name + ": attempting to assign the value of an uninitialized int signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        *_int_value = std::round(value);
        _assigned = true;
    }

    operator int() const {return get();}
};

class BoolSignalInfo : public ValueSignalInfo
{
    friend class Model;

protected:
    BoolSignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index) : ValueSignalInfo(full_name, index, vi_index)
    {
        _bool = this;
    }

    double* _bool_value{nullptr};

public:
    bool get() const
    {
        pooya_trace0;
        pooya_verify(_bool_value, _full_name + ": attempting to retrieve the value of an uninitialized bool signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return *_bool_value != 0;
    }

    void set(bool value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(_bool_value, _full_name + ": attempting to assign the value of an uninitialized bool signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        *_bool_value = value;
        _assigned = true;
    }

    operator bool() const {return get();}
};

class ArraySignalInfo : public FloatSignalInfo
{
    friend class Model;

protected:
    ArraySignalInfo(const std::string& full_name, std::size_t index, std::size_t vi_index, std::size_t size) :
        FloatSignalInfo(full_name, index, vi_index), _size(size)
    {
        _array = this;
    }

    const std::size_t _size;
    MappedArray _array_value{nullptr, 0};
    MappedArray _deriv_array_value{nullptr, 0};
    // ArraySignalId _deriv_sig{nullptr}; // the derivative signal if this is a state variable, nullptr otherwise

    // bool is_state_variable() const {return _deriv_sig;}

public:
    const MappedArray& get() const
    {
        pooya_trace0;
        pooya_verify(_array_value.rows() == int(_size), _full_name + ": attempting to retrieve the value of an uninitialized array signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return _array_value;
    }

    void set(const Array& value)
    {
        pooya_verify(_array_value.rows() == int(_size), _full_name + ": attempting to assign the value of an uninitialized array signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        pooya_verify(value.rows() == int(_size),
            std::string("size mismatch (id=") + _full_name + ")(" + std::to_string(_size) +
            " vs " + std::to_string(value.rows()) + ")!");
        _array_value = value;
        if (_deriv_array_value.size() == int(_size))
        {
            _deriv_array_value = value;
        }
        _assigned = true;
    }

    std::size_t size() const {return _size;}

    // operator const MappedArray&() const {return get();}
};

class BusSpec
{
    friend class BusInfo;

public:
    enum class SingleValueType
    {
        None,
        Scalar,
        Int,
        Bool,
    };

    struct WireInfo
    {
    protected:
        std::string _label;
        SingleValueType _single_value_type{SingleValueType::None};

    public:
        const BusSpec* _bus{nullptr};
        const std::size_t _array_size{0};

        // single-valued, coded_label defines the type and label:
        //   "i:label" -> int wire labeled "label"
        //   "b:label" -> bool wire labeled "label"
        //   "f:label" -> scalar wire labeled "label"
        //   "label"   -> scalar wire labeled "label"
        WireInfo(const std::string& coded_label);

        // array
        WireInfo(const std::string& label, std::size_t array_size) : _label(label), _array_size(array_size)
        {
            pooya_verify(array_size > 0, "array size cannot be zero!");
        }

        // bus
        WireInfo(const std::string& label, const BusSpec& bus) : _label(label), _bus(&bus) {}

        const std::string& label() const {return _label;}
        SingleValueType single_value_type() const {return _single_value_type;};
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
                    return wi.label() == label;
                }
            ));
    }

    bool operator==(const BusSpec& other) const {return this == &other;}
};

class BusInfo : public SignalInfo
{
    friend class Model;

public:
    const BusSpec& _spec;

protected:
    LabelSignalIdList _signals;

protected:
    void _set(std::size_t index, SignalId sig);

    BusInfo(const std::string& full_name, std::size_t index, const BusSpec& spec, LabelSignalIdList::const_iterator begin_, LabelSignalIdList::const_iterator end_) :
        SignalInfo(full_name, index), _spec(spec)
    {
        pooya_trace("fullname: " + full_name);
        pooya_verify(std::size_t(std::distance(begin_, end_)) == _spec._wires.size(), "incorrect number of signals: " + std::to_string(std::size_t(std::distance(begin_, end_))));
        _signals.reserve(_spec._wires.size());
        for(const auto& wi: _spec._wires)
            _signals.push_back({wi.label(), nullptr});
        for (auto& it = begin_; it != end_; it++)
            _set(_spec.index_of(it->first), it->second);
#if !defined(NDEBUG)
        for (const auto& ls: _signals)
        {
            pooya_verify(ls.second, "Unassigned wire detected: " + ls.first);
        }
#endif // !defined(NDEBUG)
        _bus = this;
    }

public:
    const BusSpec& spec() const {return _spec;}
    std::size_t size() const {return _signals.size();}
    LabelSignalIdList::const_iterator begin() const noexcept {return _signals.begin();}
    LabelSignalIdList::const_iterator end() const noexcept {return _signals.end();}

    const LabelSignalId& operator[](std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        pooya_verify(index < _signals.size(), "index out of range!");
        return _signals[index];
    }

    const LabelSignalId& at(std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        return _signals.at(index);
    }

    SignalId operator[](const std::string& label) const;
    SignalId at(const std::string& label) const;

    ValueSignalId value_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        SignalId sig = at(label);
        pooya_verify_value_signal(sig);
        return sig->as_value();
    }

    ScalarSignalId scalar_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        SignalId sig = at(label);
        pooya_verify_scalar_signal(sig);
        return sig->as_scalar();
    }

    IntSignalId int_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        SignalId sig = at(label);
        pooya_verify_int_signal(sig);
        return sig->as_int();
    }

    BoolSignalId bool_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        SignalId sig = at(label);
        pooya_verify_bool_signal(sig);
        return sig->as_bool();
    }

    ArraySignalId array_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        SignalId sig = at(label);
        pooya_verify_array_signal(sig);
        return sig->as_array();
    }

    BusId bus_at(const std::string& label) const
    {
        pooya_trace("label: " + label);
        SignalId sig = at(label);
        pooya_verify_bus(sig);
        return sig->as_bus();
    }

    ValueSignalId value_at(std::size_t index) const
    {
        pooya_trace("index: " + index);
        SignalId sig = at(index).second;
        pooya_verify_value_signal(sig);
        return sig->as_value();
    }

    ScalarSignalId scalar_at(std::size_t index) const
    {
        pooya_trace("index: " + index);
        SignalId sig = at(index).second;
        pooya_verify_scalar_signal(sig);
        return sig->as_scalar();
    }

    IntSignalId int_at(std::size_t index) const
    {
        pooya_trace("index: " + index);
        SignalId sig = at(index).second;
        pooya_verify_int_signal(sig);
        return sig->as_int();
    }

    BoolSignalId bool_at(std::size_t index) const
    {
        pooya_trace("index: " + index);
        SignalId sig = at(index).second;
        pooya_verify_bool_signal(sig);
        return sig->as_bool();
    }

    ArraySignalId array_at(std::size_t index) const
    {
        pooya_trace("index: " + index);
        SignalId sig = at(index).second;
        pooya_verify_array_signal(sig);
        return sig->as_array();
    }

    BusId bus_at(std::size_t index) const
    {
        pooya_trace("index: " + index);
        SignalId sig = at(index).second;
        pooya_verify_bus(sig);
        return sig->as_bus();
    }
};

template<typename T>
struct Types
{
};

template<>
struct Types<Array>
{
    using SignalInfo = ArraySignalInfo;
    using SignalId = ArraySignalId;
    using GetValue = const MappedArray&;
    using SetValue = const Array&;
    static void verify_signal_type([[maybe_unused]] pooya::SignalId sig) {pooya_verify_array_signal(sig);}
    static void verify_signal_type([[maybe_unused]] pooya::SignalId sig, [[maybe_unused]] std::size_t size) {pooya_verify_array_signal_size(sig, size);}
    static SignalId as_type(pooya::SignalId sig) {return sig->as_array();}
};

template<>
struct Types<double>
{
    using SignalInfo = ScalarSignalInfo;
    using SignalId = ScalarSignalId;
    using GetValue = double;
    using SetValue = double;
    static void verify_signal_type([[maybe_unused]] pooya::SignalId sig) {pooya_verify_scalar_signal(sig);}
    static SignalId as_type(pooya::SignalId sig) {return sig->as_scalar();}
};

template<>
struct Types<int>
{
    using SignalInfo = IntSignalInfo;
    using SignalId = IntSignalId;
    using GetValue = int;
    using SetValue = int;
    static void verify_signal_type([[maybe_unused]] pooya::SignalId sig) {pooya_verify_int_signal(sig);}
    static SignalId as_type(pooya::SignalId sig) {return sig->as_int();}
};

template<>
struct Types<bool>
{
    using SignalInfo = BoolSignalInfo;
    using SignalId = BoolSignalId;
    using GetValue = bool;
    using SetValue = bool;
    static void verify_signal_type([[maybe_unused]] pooya::SignalId sig) {pooya_verify_bool_signal(sig);}
    static SignalId as_type(pooya::SignalId sig) {return sig->as_bool();}
};

template<>
struct Types<BusSpec>
{
    using SignalInfo = BusInfo;
    using SignalId = BusId;
    static void verify_signal_type([[maybe_unused]] pooya::SignalId sig, [[maybe_unused]] const BusSpec& spec) {pooya_verify_bus_spec(sig, spec);}
    static SignalId as_type(pooya::SignalId sig) {return sig->as_bus();}
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
        pooya_verify(_is_scalar, "attempting to retrieve an array as a scalar!");
        return _scalar;
    }

    const Array& as_array() const
    {
        pooya_trace0;
        pooya_verify(!_is_scalar, "attempting to retrieve a scalar as an array!");
        return _array;
    }
};

// struct ValueInfo
// {
//     friend class Values;

// public:
//     const ValueSignalInfo* _si;                          // corresponding signal info

// protected:
//     // bool _assigned{false};                               // has the value been assigned?

//     // double* _scalar{nullptr};                            // valid if this is a scalar signal (bool, int, double), nullptr otherwise
//     bool _int{false};                                    // used only if this is a scalar signal, false otherwise
//     bool _bool{false};                                   // used only if this is a scalar signal, false otherwise
//     MappedArray _array{nullptr, 0};       // used only if this is an array signal, empty otherwise

//     // double* _deriv_scalar{nullptr};                      // only valid if _is_deriv member of the signal info is true and this is a float scalar signal, nullptr otherwise
//     MappedArray _deriv_array{nullptr, 0}; // used only if _is_deriv member of the signal info is true and this is an array signal, empty and unused otherwise

//     // float scalar or array
//     ValueInfo(const ValueSignalInfo& si, double* data, std::size_t float_size) :
//         _si(&si)/*, _scalar(float_size == 0 ? data : nullptr)*/, _array(float_size == 0 ? nullptr : data, float_size) {}

//     // int scalar
//     ValueInfo(const IntSignalInfo& si, double* data) :
//         _si(&si)/*, _scalar(data)*/, _int(true) {}

//     // bool scalar
//     ValueInfo(const BoolSignalInfo& si, double* data) :
//         _si(&si)/*, _scalar(data)*/, _bool(true) {}

// public:
//     // template<typename T>
//     // typename Types<T>::GetValue get() const
//     // {
//     //     pooya_trace0;
//     //     pooya_verify(_scalar == nullptr, _si->_full_name + ": attempting to retrieve a scalar as an array!");
//     //     pooya_verify(_assigned, _si->_full_name + ": attempting to access an unassigned value!");
//     //     return _array;
//     // }

//     // template<typename T>
//     // void set(typename Types<T>::SetValue value)
//     // {
//     //     pooya_trace0;
//     //     pooya_verify(_scalar == nullptr, _si->_full_name + ": cannot assign non-scalar to scalar!");
//     //     pooya_verify(!_assigned, _si->_full_name + ": re-assignment is prohibited!");
//     //     pooya_verify(_array.rows() == value.rows(),
//     //         std::string("size mismatch (id=") + _si->_full_name + ")(" + std::to_string(_array.rows()) +
//     //         " vs " + std::to_string(value.rows()) + ")!");
//     //     _array = value;
//     //     if (_si->is_deriv())
//     //         _deriv_array = value;
//     //     _assigned = true;
//     // }

//     // bool is_assigned() const {return _assigned;}
//     // bool is_scalar() const {return _scalar != nullptr;}
//     // bool is_int() const {return _int && is_scalar();}
//     // bool is_bool() const {return _bool && is_scalar();}
//     // std::size_t size() const
//     // {
//     //     return _scalar == nullptr ? _array.size() : 0;
//     // }
// };

// template<>
// inline typename Types<double>::GetValue ValueInfo::get<double>() const
// {
//     pooya_trace0;
//     pooya_verify(is_scalar(), _si->_full_name + ": attempting to retrieve an array as a scalar!");
//     pooya_verify(is_assigned(), _si->_full_name + ": attempting to access an unassigned value!");
//     return *_scalar;
// }

// template<>
// inline typename Types<int>::GetValue ValueInfo::get<int>() const
// {
//     pooya_trace0;
//     pooya_verify(is_scalar(), _si->_full_name + ": attempting to retrieve an array as an int!");
//     pooya_verify(is_int(), _si->_full_name + ": attempting to retrieve a non-int as an int!");
//     pooya_verify(is_assigned(), _si->_full_name + ": attempting to access an unassigned value!");
//     return std::round(*_scalar);
// }

// template<>
// inline typename Types<bool>::GetValue ValueInfo::get<bool>() const
// {
//     pooya_trace0;
//     pooya_verify(is_scalar(), _si->_full_name + ": attempting to retrieve an array as a bool!");
//     pooya_verify(is_bool(), _si->_full_name + ": attempting to retrieve a non-bool as a bool!");
//     pooya_verify(is_assigned(), _si->_full_name + ": attempting to access an unassigned value!");
//     return *_scalar != 0;
// }

// template<>
// inline void ValueInfo::set<int>(typename Types<int>::SetValue value)
// {
//     pooya_trace("value: " + std::to_string(value));
//     pooya_verify(is_scalar(), _si->_full_name + ": cannot assign an int to a non-scalar!");
//     pooya_verify(is_int(), _si->_full_name + ": cannot assign an int to a non-int!");
//     pooya_verify(!is_assigned(), _si->_full_name + ": re-assignment is prohibited!");
//     *_scalar = value;
//     _assigned = true;
// }

// template<>
// inline void ValueInfo::set<bool>(typename Types<bool>::SetValue value)
// {
//     pooya_trace("value: " + std::to_string(value));
//     pooya_verify(is_scalar(), _si->_full_name + ": cannot assign a bool to a non-scalar!");
//     pooya_verify(is_bool(), _si->_full_name + ": cannot assign a bool to a non-bool!");
//     pooya_verify(!is_assigned(), _si->_full_name + ": re-assignment is prohibited!");
//     *_scalar = value;
//     _assigned = true;
// }

// template<>
// inline void ValueInfo::set<double>(typename Types<double>::SetValue value)
// {
//     pooya_trace("value: " + std::to_string(value));
//     if (is_int())
//     {
//         set<int>(std::round(value));
//         return;
//     }
//     else if (is_bool())
//     {
//         set<bool>(value != 0);
//         return;
//     }

//     pooya_verify(is_scalar(), _si->_full_name + ": cannot assign a scalar to a non-scalar!");
//     pooya_verify(!is_assigned(), _si->_full_name + ": re-assignment is prohibited!");
//     *_scalar = value;
//     if (_si->is_deriv())
//         *_deriv_scalar = value;
//     _assigned = true;
// }

class ValuesArray
{
    friend class Model;

public:
    using Values = Array;
    using StateVariables = MappedArray;
    using StateVariableDerivs = Array;

protected:
    // std::vector<ValueInfo> _value_infos;
    // std::size_t             _total_size{0};
    Values _values;
    StateVariables _state_variables{nullptr, 0};
    StateVariableDerivs _state_variable_derivs;

    // inline ValueInfo& get_value_info(SignalId sig)
    // {
    //     pooya_trace0;
    //     pooya_verify_value_signal(sig);
    //     return _value_infos[sig->as_value()->_vi_index];
    // }

public:
    ValuesArray() {}
    // Values(const Values&) = delete; // forbid copy constructor

    // Values& operator=(const Values&) = delete; // forbid assignment

    // inline const ValueInfo& get_value_info(SignalId sig) const
    // {
    //     pooya_trace0;
    //     pooya_verify(sig, "invalid signal!");
    //     pooya_verify(sig->as_value(), sig->_full_name + ": value signal needed!");
    //     return _value_infos[sig->as_value()->_vi_index];
    // }

    // bool valid(SignalId sig) const
    // {
    //     pooya_trace0;
    //     return get_value_info(sig).is_assigned();
    // }

    void init(std::size_t num_values, std::size_t num_state_variables);

    std::size_t num_state_variables() const {return _state_variables.size();}
    // const decltype(_value_infos)& value_infos() const {return _value_infos;}
    Values& values() {return _values;}
    const Values& values() const {return _values;}
// #ifdef POOYA_NDEBUG
    const StateVariables& state_variables() const {return _state_variables;}
// #else
//     const decltype(_state_variables)& state_variables() const;
// #endif // definded(POOYA_NDEBUG)
    StateVariableDerivs& state_variable_derivs() {return _state_variable_derivs;}
    const StateVariableDerivs& state_variable_derivs() const {return _state_variable_derivs;}

    // template<typename T>
    // typename Types<T>::GetValue get(SignalId sig) const
    // {
    //     pooya_trace0;
    //     return get_value_info(sig).get<T>();
    // }

    // typename Types<double>::GetValue get(ScalarSignalId sig) const {return get<double>(sig);}
    // typename Types<int   >::GetValue get(IntSignalId    sig) const {return get<int   >(sig);}
    // typename Types<bool  >::GetValue get(BoolSignalId   sig) const {return get<bool  >(sig);}
    // typename Types<Array >::GetValue get(ArraySignalId  sig) const {return get<Array >(sig);}

    // typename Types<double>::GetValue get_as_scalar(SignalId sig) const;

    // typename Types<double>::GetValue operator[](ScalarSignalId sig) const {return get<double>(sig);}
    // typename Types<int   >::GetValue operator[](IntSignalId    sig) const {return get<int   >(sig);}
    // typename Types<bool  >::GetValue operator[](BoolSignalId   sig) const {return get<bool  >(sig);}
    // typename Types<Array >::GetValue operator[](ArraySignalId  sig) const {return get<Array >(sig);}

    // template<typename T>
    // void set(SignalId sig, typename Types<T>::SetValue value)
    // {
    //     pooya_trace0;
    //     get_value_info(sig).set<T>(value);
    // }

    // void set(ScalarSignalId sig, double value) {set<double>(sig, value);}
    // void set(IntSignalId sig, double value) {set<int>(sig, std::round(value));} // avoid the default implicit double-to-int conversion
    // void set(BoolSignalId sig, bool value) {set<bool>(sig, value);}
    // void set(ArraySignalId sig, const Array& value) {set<Array>(sig, value);}

    // template<typename T, typename GetValue = typename Types<T>::GetValue, typename SetValue = typename Types<T>::SetValue>
    // class ValueInfoWrapper
    // {
    //     friend class Values;
    //     ValueInfo& _vi;
    //     ValueInfoWrapper(ValueInfo& vi) : _vi(vi) {}
    // public:
    //     void operator=(SetValue value)
    //     {
    //         pooya_trace0;
    //         _vi.set<T>(value);
    //     }
    //     operator GetValue() const
    //     {
    //         pooya_trace0;
    //         return _vi.get<T>();
    //     }
    // };

    // ValueInfoWrapper<double> operator[](ScalarSignalId sig) {return Values::ValueInfoWrapper<double>(get_value_info(sig));}
    // ValueInfoWrapper<int, typename Types<int>::GetValue, typename Types<double>::SetValue> operator[](IntSignalId sig) {return Values::ValueInfoWrapper<int, typename Types<int>::GetValue, typename Types<double>::SetValue>(get_value_info(sig));}
    // ValueInfoWrapper<bool> operator[](BoolSignalId   sig) {return Values::ValueInfoWrapper<bool>  (get_value_info(sig));}
    // ValueInfoWrapper<Array> operator[](ArraySignalId  sig) {return Values::ValueInfoWrapper<Array> (get_value_info(sig));}

    // void invalidate();
    // void reset_with_state_variables(const Eigen::ArrayXd& state_variables);

    // void stream(std::ostream& os) const
    // {
    //     pooya_trace0;
    //     int k = 0;
    //     for (const auto& vi: _value_infos)
    //     {
    //         os << "- [" << k++ << "]: ";
    //         (vi.is_assigned() ? (vi._scalar ? os << *vi._scalar : os << vi._array) : os << "*") << "\n";
    //     }
    //     os << "\n";
    // }
};

// template<>
// inline void Values::ValueInfoWrapper<int, typename Types<int>::GetValue, typename Types<double>::SetValue>::operator=(typename Types<double>::SetValue value)
// {
//     pooya_trace0;
//     _vi.set<int>(std::round(value));
// }

// inline std::ostream& operator<<(std::ostream& os, const Values& values)
// {
//     pooya_trace0;
//     values.stream(os);
//     return os;
// }

}

#endif // __POOYA_SIGNAL_HPP__
