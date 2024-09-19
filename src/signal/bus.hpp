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

#include "scalar_signal.hpp"
#include <algorithm>
#include <string>
#include <vector>

#if defined(POOYA_USE_SMART_PTRS)
#include <optional>
#endif // defined(POOYA_USE_SMART_PTRS)

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "src/signal/signal.hpp"
#include "src/signal/signal_id.hpp"
#include "src/signal/label_signal.hpp"

#define pooya_verify_bus(sig) \
    pooya_verify(sig.is_bus(), sig.name() + ": bus signal expected!");

#define pooya_verify_bus_spec(sig, spec_) \
    pooya_verify_bus(sig); \
    pooya_verify((sig)->as_bus()->spec() == spec_, (sig)->_full_name + ": bus spec mismatch!");

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
        // Int,
        // Bool,
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
        // WireInfo(const std::string& label, std::size_t array_size) : _label(label), _array_size(array_size)
        // {
        //     pooya_verify(array_size > 0, "array size cannot be zero!");
        // }

        // bus
        WireInfo(const std::string& label, const BusSpec& bus) : _label(label)
            , _bus(bus) {}

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

// #if defined(POOYA_USE_SMART_PTRS)

// inline BusId SignalInfo::as_bus() {return _bus ? std::static_pointer_cast<BusInfo>(shared_from_this()) : BusId();}
// inline ReadOnlyBusId SignalInfo::as_bus() const {return _bus ? std::static_pointer_cast<const BusInfo>(shared_from_this()) : ReadOnlyBusId();}

// #else // defined(POOYA_USE_SMART_PTS)

// inline BusId SignalInfo::as_bus() {return _bus ? static_cast<BusId>(this) : nullptr;}
// inline ReadOnlyBusId SignalInfo::as_bus() const {return _bus ? static_cast<ReadOnlyBusId>(this) : nullptr;}

// #endif // defined(POOYA_USE_SMART_PTS)

class Bus : public Signal
{
    friend class Parent;
protected:
    struct BusInfo : public Signal::SignalInfo
    {
        // static const BusSpec _empty_spec{}; // used by the default constructor
        std::reference_wrapper<const BusSpec> _spec;
        LabelSignalList _signals;

    // protected:
        // void _set(std::size_t index, Signal& sig);

    // public:
        // BusInfo() : SignalInfo(), _spec(_empty_spec) {}

        BusInfo(const BusSpec& spec, LabelSignalList::const_iterator begin_, LabelSignalList::const_iterator end_)
            : SignalInfo(), _spec(spec)
        {
            // pooya_trace("fullname: " + full_name);
            pooya_verify(std::size_t(std::distance(begin_, end_)) == spec._wires.size(), "incorrect number of signals: " + std::to_string(std::size_t(std::distance(begin_, end_))));
            _signals.reserve(spec._wires.size());
            for(const auto& wi: spec._wires)
            {
                auto iter = std::find_if(begin_, end_,
                    [&](const LabelSignal& ls) -> bool {return wi.label() == ls.first;});
                pooya_verify(iter != end_, "Unassigned wire detected: " + wi.label());
                _signals.push_back({wi.label(), iter->second});
            }
    //         for (auto& it = begin_; it != end_; it++)
    //             {_set(spec.index_of(it->first), it->second);}
    //             _signals.push_back({wi.label(), Signal()});
    //         }
    //         for (auto& it = begin_; it != end_; it++)
    //             {_set(spec.index_of(it->first), it->second);}
    // #if defined(POOYA_DEBUG)
    //         for (const auto& ls: _signals)
    //         {
    //             pooya_verify(ls.second, "Unassigned wire detected: " + ls.first);
    //         }
    // #endif // defined(POOYA_DEBUG)

            // _bus = true;
        }

        // static std::shared_ptr<BusInfo> create_new(const std::string& full_name)
        // {
        //     auto ret = std::make_shared<BusInfo>();
        //     if (!ret) {helper::pooya_throw_exception(__FILE__, __LINE__, "Failed to create a new BusInfo!");}
        //     ret->_full_name = full_name;
        //     return ret;
        // } 

        static std::shared_ptr<BusInfo> create_new(const std::string& full_name, const BusSpec& spec, LabelSignalList::const_iterator begin_, LabelSignalList::const_iterator end_)
        {
            auto ret = std::make_shared<BusInfo>(spec, begin_, end_);
            if (!ret) {helper::pooya_throw_exception(__FILE__, __LINE__, "Failed to create a new BusInfo!");}
            ret->_full_name = full_name;
            return ret;
        } 

        // const BusSpec& spec() const {return _spec;}
        // std::size_t size() const {return _signals.size();}
        // LabelSignalList::const_iterator begin() const noexcept {return _signals.begin();}
        // LabelSignalList::const_iterator end() const noexcept {return _signals.end();}

        // const LabelSignal& operator[](std::size_t index) const
        // {
        //     pooya_trace("index: " + std::to_string(index));
        //     pooya_verify(index < _signals.size(), "index out of range!");
        //     return _signals[index];
        // }

        // const LabelSignal& at(std::size_t index) const
        // {
        //     pooya_trace("index: " + std::to_string(index));
        //     return _signals.at(index);
        // }

        // const LabelSignal::second_type& operator[](const std::string& label) const;
        // const LabelSignal::second_type& at(const std::string& label) const;
        // ValueSignalId value_at(const std::string& label) const;
        // ScalarSignalRef scalar_at(const std::string& label) const;
        // IntSignalId int_at(const std::string& label) const;
        // BoolSignalId bool_at(const std::string& label) const;
        // ArraySignalId array_at(const std::string& label) const;
        // BusId bus_at(const std::string& label) const;
        // ValueSignalId value_at(std::size_t index) const;
        // ScalarSignalConstRef scalar_at(std::size_t index) const;
        // IntSignalId int_at(std::size_t index) const;
        // BoolSignalId bool_at(std::size_t index) const;
        // ArraySignalId array_at(std::size_t index) const;
        // BusId bus_at(std::size_t index) const;
    };

    std::shared_ptr<BusInfo> impl_;

public:
    // Bus(const std::string& given_name="") :
    //     Signal(*BusInfo::create_new(given_name)),
    //     impl_(std::static_pointer_cast<BusInfo>(signal_impl_.get().shared_from_this()))
    // {
    //     _bus = *this;
    // }
    Bus(const std::string& given_name, const BusSpec& spec, LabelSignalList::const_iterator begin_, LabelSignalList::const_iterator end_) :
        Signal(*BusInfo::create_new(given_name, spec, begin_, end_)),
        impl_(std::static_pointer_cast<BusInfo>(signal_impl_.get().shared_from_this()))
    {
        _bus = *this;
    }
    Bus(const Bus& bus) :
        Signal(bus),
        impl_(std::static_pointer_cast<BusInfo>(signal_impl_.get().shared_from_this()))
    {
        _bus = *this;
    }

    // const LabelSignal& at(std::size_t index) const
    // {
    //     pooya_trace("index: " + std::to_string(index));
    //     return impl_->_signals.at(index);
    // }

    Bus& operator=(const Bus& bus) = default;

    template<typename T>
    const typename Types<T>::Signal& at(std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        return std::get<typename Types<T>::SignalConstRef>(impl_->_signals.at(index).second);
    }

    template<typename T>
    const typename Types<T>::Signal& at(const std::string& label) const;

    // ScalarSignalInfo& operator->() {return *impl_;}
    // double get() const {return impl_->get();}
    // void set(double v) {impl_->set(v);}

    // std::shared_ptr<BusInfo> operator->() // TEMPORARY
    // {
    //     return std::static_pointer_cast<BusInfo>(impl_->shared_from_this());
    // }
    // std::shared_ptr<const BusInfo> operator->() const // TEMPORARY
    // {
    //     return std::static_pointer_cast<const BusInfo>(impl_->shared_from_this());
    // }
    // std::shared_ptr<ScalarSignalInfo> id() // TEMPORARY
    // {
    //     return std::static_pointer_cast<ScalarSignalInfo>(impl_->shared_from_this());
    // }
    // operator SignalId()
    // {
    //     return impl_->shared_from_this();
    // }
    // const std::string& name() const override {return impl_->_full_name;}

    // const BusSpec& spec() const {return _spec;}
    std::size_t size() const {return impl_->_signals.size();}
    LabelSignalList::const_iterator begin() const noexcept {return impl_->_signals.begin();}
    LabelSignalList::const_iterator end() const noexcept {return impl_->_signals.end();}

    // const LabelSignal& operator[](std::size_t index) const
    // {
    //     pooya_trace("index: " + std::to_string(index));
    //     pooya_verify(index < _signals.size(), "index out of range!");
    //     return _signals[index];
    // }

    // const LabelSignal& at(std::size_t index) const
    // {
    //     pooya_trace("index: " + std::to_string(index));
    //     return _signals.at(index);
    // }

    // const LabelSignal::second_type& operator[](const std::string& label) const;
    // const LabelSignal::second_type& at(const std::string& label) const;
    // ValueSignalId value_at(const std::string& label) const;
    // ScalarSignalRef scalar_at(const std::string& label) const;
    // IntSignalId int_at(const std::string& label) const;
    // BoolSignalId bool_at(const std::string& label) const;
    // ArraySignalId array_at(const std::string& label) const;
    // BusId bus_at(const std::string& label) const;
    // ValueSignalId value_at(std::size_t index) const;
    // ScalarSignalConstRef scalar_at(std::size_t index) const;
    // IntSignalId int_at(std::size_t index) const;
    // BoolSignalId bool_at(std::size_t index) const;
    // ArraySignalId array_at(std::size_t index) const;
    // BusId bus_at(std::size_t index) const;
};

// using BusRef = std::reference_wrapper<Bus>;
// using BusConstRef = std::reference_wrapper<const Bus>;

}

#endif // __POOYA_SIGNAL_BUS_HPP__
