/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SIGNAL_BUS_HPP__
#define __POOYA_SIGNAL_BUS_HPP__

#include <algorithm>
#include <string>
#include <vector>
#include <optional>

#include "int_signal.hpp"
#include "scalar_signal.hpp"
#include "array_signal.hpp"
#include "bool_signal.hpp"
#include "signal_id.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "src/signal/signal.hpp"
#include "src/signal/signal_id.hpp"
#include "src/signal/label_signal.hpp"

#define pooya_verify_bus(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_bus(), (sig)->_full_name + ": bus signal expected!");

#define pooya_verify_bus_spec(sig, spec_) \
    pooya_verify_bus(sig); \
    pooya_verify((sig)->as_bus().spec() == spec_, (sig)->_full_name + ": bus spec mismatch!");

namespace pooya
{

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
        std::optional<std::reference_wrapper<const BusSpec>> _bus;
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
        WireInfo(const std::string& label, const BusSpec& bus) : _label(label), _bus(bus) {}

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
        {
            if (wi._bus)
                {ret += wi._bus.value().get().total_size();}
        }
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
    std::reference_wrapper<const BusSpec> _spec;

protected:
    LabelSignalIdList _signals;

    void _set(std::size_t index, SignalId sig);

public:
    template<typename Iter>
    BusInfo(Protected, const std::string& full_name, const BusSpec& spec, Iter begin_, Iter end_)
        : SignalInfo(full_name, BusType), _spec(spec)
    {
        pooya_trace("fullname: " + full_name);
        pooya_verify(std::size_t(std::distance(begin_, end_)) == spec._wires.size(), "incorrect number of signals: " + std::to_string(std::size_t(std::distance(begin_, end_))));
        _signals.reserve(spec._wires.size());
        for(const auto& wi: spec._wires)
            {_signals.push_back({wi.label(), SignalId()});}
        for (auto& it = begin_; it != end_; it++)
            {_set(spec.index_of(it->first), it->second);}
#if defined(POOYA_DEBUG)
        for (const auto& ls: _signals)
        {
            pooya_verify(ls.second, "Unassigned wire detected: " + ls.first);
        }
#endif // defined(POOYA_DEBUG)
    }

    // static BusId create_new(const std::string& full_name, const BusSpec& spec, LabelSignalIdList::const_iterator begin_, LabelSignalIdList::const_iterator end_)
    // {
    //     return std::make_shared<BusInfo>(Protected(), full_name, spec, begin_, end_);
    // } 

    template<typename Iter>
    static BusId create_new(const std::string& full_name, const BusSpec& spec, Iter begin_, Iter end_)
    {
        pooya_trace("bus: " + full_name);
        auto size = spec._wires.size();
        pooya_verify(std::distance(begin_, end_) <= int(size), "Too many entries in the initializer list!");

        std::vector<LabelSignalId> label_signals;
        label_signals.reserve(size);
        for (const auto& wi: spec._wires) {label_signals.push_back({wi.label(), SignalId()});}
        for (auto it=begin_; it != end_; it++)
        {
            auto index = spec.index_of(it->first);
            pooya_verify(!label_signals.at(index).second, std::string("Duplicate label: ") + it->first);
            label_signals.at(index).second = it->second;
        }
        auto wit = spec._wires.begin();
        for (auto& ls: label_signals)
        {
            if (!ls.second)
            {
                std::string name = full_name + "." + wit->label();
                if (wit->_bus)
                    // {ls.second = create_bus(name, *wit->_bus);}
                    {ls.second = BusInfo::create_new(name, (*wit->_bus).get());}
                else if (wit->_array_size > 0)
                    // {ls.second = create_array_signal(name, wit->_array_size);}
                    {ls.second = ArraySignalInfo::create_new(name, wit->_array_size);}
                else if (wit->single_value_type() == BusSpec::SingleValueType::Scalar)
                    // {ls.second = create_scalar_signal(name);}
                    {ls.second = ScalarSignalInfo::create_new(name);}
                else if (wit->single_value_type() == BusSpec::SingleValueType::Int)
                    // {ls.second = create_int_signal(name);}
                    {ls.second = IntSignalInfo::create_new(name);}
                else if (wit->single_value_type() == BusSpec::SingleValueType::Bool)
                    // {ls.second = create_bool_signal(name);}
                    {ls.second = BoolSignalInfo::create_new(name);}
                else
                    {pooya_verify(false, name + ": unknown wire type!");}
            }
            wit++;
        }

        // return model_ref().register_bus(make_signal_name(given_name, true), spec, label_signals.begin(), label_signals.end());
        // return BusInfo::create_new(make_signal_name(given_name, true), spec, label_signals.begin(), label_signals.end());
        return std::make_shared<BusInfo>(Protected(), full_name, spec, label_signals.begin(), label_signals.end());
    }

    static BusId create_new(const std::string& full_name, const BusSpec& spec, const std::initializer_list<LabelSignalId>& l)
    {
        // pooya_trace("block: " + full_name());
        pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");
        return create_new(full_name, spec, l.begin(), l.end());
    }

    static BusId create_new(const std::string& full_name, const BusSpec& spec, const std::initializer_list<SignalId>& l={})
    {
        // pooya_trace("block: " + full_name());
        pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");

        LabelSignalIdList label_signals;
        label_signals.reserve(l.size());

        auto wit = spec._wires.begin();
        for (const auto& sig: l)
        {
            label_signals.push_back({wit->label(), sig});
            wit++;
        }

        return create_new(full_name, spec, label_signals.begin(), label_signals.end());
    }

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
    ValueSignalId value_at(const std::string& label) const;
    ScalarSignalId scalar_at(const std::string& label) const;
    IntSignalId int_at(const std::string& label) const;
    BoolSignalId bool_at(const std::string& label) const;
    ArraySignalId array_at(const std::string& label) const;
    BusId bus_at(const std::string& label) const;
    ValueSignalId value_at(std::size_t index) const;
    ScalarSignalId scalar_at(std::size_t index) const;
    IntSignalId int_at(std::size_t index) const;
    BoolSignalId bool_at(std::size_t index) const;
    ArraySignalId array_at(std::size_t index) const;
    BusId bus_at(std::size_t index) const;
};

inline BusInfo& SignalInfo::as_bus()
{
    pooya_verify(_type & BusType, "Illegal attempt to dereference a non-bus as a bus.");
    return *static_cast<BusInfo*>(this);
}

inline const BusInfo& SignalInfo::as_bus() const
{
    pooya_verify(_type & BusType, "Illegal attempt to dereference a non-bus as a bus.");
    return *static_cast<const BusInfo*>(this);
}

}

#endif // __POOYA_SIGNAL_BUS_HPP__
