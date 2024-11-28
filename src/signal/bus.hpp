/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SIGNAL_BUS_HPP__
#define __POOYA_SIGNAL_BUS_HPP__

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "array_signal.hpp"
#include "bool_signal.hpp"
#include "int_signal.hpp"
#include "label_signal.hpp"
#include "scalar_signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"

#define pooya_verify_bus(sig)                                                                                          \
    pooya_verify_valid_signal(sig);                                                                                    \
    pooya_verify((sig)->is_bus(), (sig)->name().str() + ": bus signal expected!");

#define pooya_verify_bus_spec(sig, spec_)                                                                              \
    pooya_verify_bus(sig);                                                                                             \
    pooya_verify((sig)->as_bus().spec() == spec_, (sig)->name().str() + ": bus spec mismatch!");

namespace pooya
{

class BusSpec
{
    friend class BusImpl;

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

        const std::string& label() const { return _label; }
        SingleValueType single_value_type() const { return _single_value_type; };
    };

    const std::vector<WireInfo> _wires;

    // empty
    BusSpec() = default;

    template<typename Iter>
    BusSpec(Iter begin_, Iter end_) : _wires(begin_, end_)
    {
    }

    BusSpec(const std::initializer_list<WireInfo>& l) : _wires(l) {}

    std::size_t total_size() const
    {
        pooya_trace0;
        std::size_t ret = _wires.size();
        for (const auto& wi : _wires)
        {
            if (wi._bus)
            {
                ret += wi._bus.value().get().total_size();
            }
        }
        return ret;
    }

    std::size_t index_of(const std::string& label) const
    {
        pooya_trace("label: " + label);
        return std::distance(_wires.begin(), std::find_if(_wires.begin(), _wires.end(),
                                                          [&](const WireInfo& wi) { return wi.label() == label; }));
    }

    bool operator==(const BusSpec& other) const { return this == &other; }
};

class BusImpl : public SignalImpl
{
protected:
    std::reference_wrapper<const BusSpec> _spec;
    LabelSignalImplPtrList _signals;

    void _set(std::size_t index, SignalImplPtr sig);

public:
    template<typename Iter>
    BusImpl(Protected, const ValidName& name, const BusSpec& spec, Iter begin_, Iter end_)
        : SignalImpl(name, BusType), _spec(spec)
    {
        pooya_trace("fullname: " + name.str());
        pooya_verify(std::size_t(std::distance(begin_, end_)) == spec._wires.size(),
                     "incorrect number of signals: " + std::to_string(std::size_t(std::distance(begin_, end_))));
        _signals.reserve(spec._wires.size());
        for (const auto& wi : spec._wires)
        {
            _signals.push_back({wi.label(), SignalImplPtr()});
        }
        for (auto& it = begin_; it != end_; it++)
        {
            _set(spec.index_of(it->first), it->second);
        }
#if defined(POOYA_DEBUG)
        for (const auto& ls : _signals)
        {
            pooya_verify(ls.second, "Unassigned wire detected: " + ls.first);
        }
#endif // defined(POOYA_DEBUG)
    }

    template<typename Iter>
    static BusImplPtr create_new(const ValidName& name, const BusSpec& spec, Iter begin_, Iter end_)
    {
        pooya_trace("bus: " + name.str());
        auto size = spec._wires.size();
        pooya_verify(std::distance(begin_, end_) <= int(size), "Too many entries in the initializer list!");

        std::vector<LabelSignalImplPtr> label_signals;
        label_signals.reserve(size);
        for (const auto& wi : spec._wires)
        {
            label_signals.push_back({wi.label(), SignalImplPtr()});
        }
        for (auto it = begin_; it != end_; it++)
        {
            auto index = spec.index_of(it->first);
            pooya_verify(!label_signals.at(index).second, std::string("Duplicate label: ") + it->first);
            label_signals.at(index).second = it->second;
        }
        auto wit = spec._wires.begin();
        for (auto& ls : label_signals)
        {
            if (!ls.second)
            {
                ValidName new_name(name | wit->label());
                if (wit->_bus)
                {
                    ls.second = BusImpl::create_new(new_name, (*wit->_bus).get());
                }
                else if (wit->_array_size > 0)
                {
                    ls.second = ArraySignalImpl::create_new(new_name, wit->_array_size);
                }
                else if (wit->single_value_type() == BusSpec::SingleValueType::Scalar)
                {
                    ls.second = ScalarSignalImpl::create_new(new_name);
                }
                else if (wit->single_value_type() == BusSpec::SingleValueType::Int)
                {
                    ls.second = IntSignalImpl::create_new(new_name);
                }
                else if (wit->single_value_type() == BusSpec::SingleValueType::Bool)
                {
                    ls.second = BoolSignalImpl::create_new(new_name);
                }
                else
                {
                    pooya_verify(false, new_name.str() + ": unknown wire type!");
                }
            }
            wit++;
        }

        return std::make_shared<BusImpl>(Protected(), name, spec, label_signals.begin(), label_signals.end());
    }

    template<typename Iter>
    static BusImplPtr create_new(const BusSpec& spec, Iter begin_, Iter end_)
    {
        return create_new("", spec, begin_, end_);
    }

    static BusImplPtr create_new(const ValidName& name, const BusSpec& spec,
                            const std::initializer_list<LabelSignalImplPtr>& l)
    {
        pooya_trace("create_new: " + name.str());
        pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");
        return create_new(name, spec, l.begin(), l.end());
    }

    static BusImplPtr create_new(const BusSpec& spec, const std::initializer_list<LabelSignalImplPtr>& l)
    {
        return create_new("", spec, l);
    }

    static BusImplPtr create_new(const ValidName& name, const BusSpec& spec,
                            const std::initializer_list<SignalImplPtr>& l = {})
    {
        pooya_trace("create_new: " + name.str());
        pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");

        LabelSignalImplPtrList label_signals;
        label_signals.reserve(l.size());

        auto wit = spec._wires.begin();
        for (const auto& sig : l)
        {
            label_signals.push_back({wit->label(), sig});
            wit++;
        }

        return create_new(name, spec, label_signals.begin(), label_signals.end());
    }

    static BusImplPtr create_new(const BusSpec& spec, const std::initializer_list<SignalImplPtr>& l = {})
    {
        return create_new("", spec, l);
    }

    const BusSpec& spec() const { return _spec; }
    std::size_t size() const { return _signals.size(); }
    LabelSignalImplPtrList::const_iterator begin() const noexcept { return _signals.begin(); }
    LabelSignalImplPtrList::const_iterator end() const noexcept { return _signals.end(); }

    const LabelSignalImplPtr& operator[](std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        pooya_verify(index < _signals.size(), "index out of range!");
        return _signals[index];
    }

    const LabelSignalImplPtr& at(std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        return _signals.at(index);
    }

    SignalImplPtr operator[](const std::string& label) const;
    SignalImplPtr at(const std::string& label) const;
};

inline BusImpl& SignalImpl::as_bus()
{
    pooya_verify(_type & BusType, "Illegal attempt to dereference a non-bus as a bus.");
    return *static_cast<BusImpl*>(this);
}

inline const BusImpl& SignalImpl::as_bus() const
{
    pooya_verify(_type & BusType, "Illegal attempt to dereference a non-bus as a bus.");
    return *static_cast<const BusImpl*>(this);
}

class Bus : public Signal<BusSpec>
{
    using Base = Signal<BusSpec>;

public:
    Bus(const BusSpec& spec = BusSpec(), const std::initializer_list<LabelSignalImplPtr>& l = {})
        : Base(BusImpl::create_new(spec, l))
    {
    }
    Bus(const ValidName& name, const BusSpec& spec = BusSpec(), const std::initializer_list<LabelSignalImplPtr>& l = {})
        : Base(BusImpl::create_new(name, spec, l))
    {
    }
    Bus(const BusSpec& spec, const std::initializer_list<SignalImplPtr>& l) : Base(BusImpl::create_new(spec, l)) {}
    Bus(const ValidName& name, const BusSpec& spec, const std::initializer_list<SignalImplPtr>& l)
        : Base(BusImpl::create_new(name, spec, l))
    {
    }
    template<typename Iter>
    Bus(const BusSpec& spec, Iter begin_, Iter end_) : Base(BusImpl::create_new(spec, begin_, end_))
    {
    }
    template<typename Iter>
    Bus(const ValidName& name, const BusSpec& spec, Iter begin_, Iter end_)
        : Base(BusImpl::create_new(name, spec, begin_, end_))
    {
    }
    explicit Bus(const SignalImplPtr& sid) : Base(sid && sid->is_bus() ? std::static_pointer_cast<BusImpl>(sid) : nullptr) {}
    Bus(const BusImplPtr& sid) : Base(sid) {}

    Bus& operator=(const Bus&) = delete;

    void reset(const BusSpec& spec = BusSpec(), const std::initializer_list<LabelSignalImplPtr>& l = {})
    {
        _sid = BusImpl::create_new(spec, l);
    }
    void reset(const ValidName& name, const BusSpec& spec = BusSpec(),
               const std::initializer_list<LabelSignalImplPtr>& l = {})
    {
        _sid = BusImpl::create_new(name, spec, l);
    }
    void reset(const BusSpec& spec, const std::initializer_list<SignalImplPtr>& l)
    {
        _sid = BusImpl::create_new(spec, l);
    }
    void reset(const ValidName& name, const BusSpec& spec, const std::initializer_list<SignalImplPtr>& l)
    {
        _sid = BusImpl::create_new(name, spec, l);
    }
    template<typename Iter>
    void reset(const BusSpec& spec, Iter begin_, Iter end_)
    {
        _sid = BusImpl::create_new(spec, begin_, end_);
    }
    template<typename Iter>
    void reset(const ValidName& name, const BusSpec& spec, Iter begin_, Iter end_)
    {
        _sid = BusImpl::create_new(name, spec, begin_, end_);
    }

    const BusSpec& spec() const { return _sid->spec(); }
    std::size_t size() const { return _sid->size(); }
    LabelSignalImplPtrList::const_iterator begin() const noexcept { return _sid->begin(); }
    LabelSignalImplPtrList::const_iterator end() const noexcept { return _sid->end(); }
    const LabelSignalImplPtr& operator[](std::size_t index) const { return (*_sid)[index]; }
    const LabelSignalImplPtr& at(std::size_t index) const { return _sid->at(index); }
    SignalImplPtr operator[](const std::string& label) const { return (*_sid)[label]; }
    SignalImplPtr at(const std::string& label) const { return _sid->at(label); }

    ValueSignalImplPtr value_at(const std::string& label) const;
    FloatSignalImplPtr float_at(const std::string& label) const;
    ScalarSignalImplPtr scalar_at(const std::string& label) const;
    IntSignalImplPtr int_at(const std::string& label) const;
    BoolSignalImplPtr bool_at(const std::string& label) const;
    ArraySignalImplPtr array_at(const std::string& label) const;
    BusImplPtr bus_at(const std::string& label) const;
    ValueSignalImplPtr value_at(std::size_t index) const;
    FloatSignalImplPtr float_at(std::size_t index) const;
    ScalarSignalImplPtr scalar_at(std::size_t index) const;
    IntSignalImplPtr int_at(std::size_t index) const;
    BoolSignalImplPtr bool_at(std::size_t index) const;
    ArraySignalImplPtr array_at(std::size_t index) const;
    BusImplPtr bus_at(std::size_t index) const;

    using Signal<BusSpec>::reset;
};

} // namespace pooya

#endif // __POOYA_SIGNAL_BUS_HPP__
