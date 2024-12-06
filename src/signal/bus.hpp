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

#include <string>
#include <unordered_map>
#include <vector>

#include "signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"

#define pooya_verify_bus(sig)                                                                                          \
    pooya_verify_valid_signal(sig);                                                                                    \
    pooya_verify((sig)->is_bus(), (sig)->name().str() + ": bus signal expected!");

namespace pooya
{

class BusSpec
{
};

class BusImpl : public SignalImpl
{
public:
    using LabelSignalImplPtr = std::pair<std::string, SignalImplPtr>;
    using Signals            = std::unordered_map<std::string, SignalImplPtr>;
    using OrderedKeys        = std::vector<std::string>;

protected:
    Signals _signals;
    OrderedKeys _ordered_keys;

    static std::string _make_auto_label(std::size_t index) { return "sig" + std::to_string(index); }

public:
    template<typename Iter>
    BusImpl(Protected, const ValidName& name, Iter begin_, Iter end_) : SignalImpl(name, BusType)
    {
        pooya_trace("name: " + name.str());

        auto size = std::distance(begin_, end_);
        _signals.reserve(size);
        _ordered_keys.reserve(size);

        for (auto& it = begin_; it != end_; it++)
        {
            if (!it->second) continue;

            auto label = ValidName(it->first).str();
            if (label.empty()) continue;

            _signals.insert({label, it->second});
            _ordered_keys.push_back(label);
        }

        _ordered_keys.shrink_to_fit();
    }

    template<typename Iter>
    static BusImplPtr create_new(const ValidName& name, Iter begin_, Iter end_)
    {
        pooya_trace("bus: " + name.str());
        return std::make_shared<BusImpl>(Protected(), name, begin_, end_);
    }

    template<typename Iter>
    static BusImplPtr create_new(Iter begin_, Iter end_)
    {
        return create_new("", begin_, end_);
    }

    static BusImplPtr create_new(const ValidName& name, const std::initializer_list<SignalImplPtr>& l = {})
    {
        pooya_trace("create_new: " + name.str());

        std::vector<LabelSignalImplPtr> signals;
        signals.reserve(l.size());
        int k{0};
        for (auto& sig : l)
        {
            signals.emplace_back(_make_auto_label(k++), sig);
        }

        return create_new(name, signals.begin(), signals.end());
    }

    static BusImplPtr create_new(const std::initializer_list<SignalImplPtr>& l = {}) { return create_new("", l); }

    template<typename T>
    static BusImplPtr create_new(const T& signal)
    {
        std::vector<LabelSignalImplPtr> label_signal({{_make_auto_label(0), signal->shared_from_this()}});
        return create_new(label_signal.begin(), label_signal.end());
    }

    static BusImplPtr create_new(const ValidName& name, const std::initializer_list<LabelSignalImplPtr>& l)
    {
        return create_new(name, l.begin(), l.end());
    }

    static BusImplPtr create_new(const std::initializer_list<LabelSignalImplPtr>& l) { return create_new("", l); }

    std::size_t size() const { return _signals.size(); }
    OrderedKeys::const_iterator begin() const noexcept { return _ordered_keys.begin(); }
    OrderedKeys::const_iterator end() const noexcept { return _ordered_keys.end(); }

    const SignalImplPtr& at(std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        return _signals.at(_ordered_keys.at(index));
    }

    const SignalImplPtr& operator[](std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        pooya_verify(index < _signals.size(), "index out of range!");
        return _signals.at(_ordered_keys[index]);
    }

    const SignalImplPtr& at(const std::string& label) const;

    const SignalImplPtr& operator[](const std::string& label) const { return at(label); }
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
    Bus(const std::initializer_list<SignalImplPtr>& l = {}) : Base(BusImpl::create_new(l)) {}
    Bus(const ValidName& name, const std::initializer_list<SignalImplPtr>& l = {}) : Base(BusImpl::create_new(name, l))
    {
    }
    Bus(const std::initializer_list<BusImpl::LabelSignalImplPtr>& l) : Base(BusImpl::create_new(l)) {}
    Bus(const ValidName& name, const std::initializer_list<BusImpl::LabelSignalImplPtr>& l)
        : Base(BusImpl::create_new(name, l))
    {
    }
    Bus(BusImpl::Signals::const_iterator begin_, BusImpl::Signals::const_iterator end_)
        : Base(BusImpl::create_new(begin_, end_))
    {
    }
    Bus(const ValidName& name, BusImpl::Signals::const_iterator begin_, BusImpl::Signals::const_iterator end_)
        : Base(BusImpl::create_new(name, begin_, end_))
    {
    }
    template<typename T>
    Bus(const T& signal) : Base(BusImpl::create_new(signal))
    {
    }
    explicit Bus(const SignalImplPtr& sid)
        : Base(sid && sid->is_bus() ? std::static_pointer_cast<BusImpl>(sid) : nullptr)
    {
    }
    Bus(const BusImplPtr& sid) : Base(sid) {}

    Bus& operator=(const Bus&) = delete;

    void reset(const std::initializer_list<BusImpl::LabelSignalImplPtr>& l) { _sid = BusImpl::create_new(l); }
    void reset(const ValidName& name, const std::initializer_list<BusImpl::LabelSignalImplPtr>& l)
    {
        _sid = BusImpl::create_new(name, l);
    }
    void reset(const std::initializer_list<SignalImplPtr>& l = {}) { _sid = BusImpl::create_new(l); }
    void reset(const ValidName& name, const std::initializer_list<SignalImplPtr>& l = {})
    {
        _sid = BusImpl::create_new(name, l);
    }
    void reset(BusImpl::Signals::const_iterator begin_, BusImpl::Signals::const_iterator end_)
    {
        _sid = BusImpl::create_new(begin_, end_);
    }
    void reset(const ValidName& name, BusImpl::Signals::const_iterator begin_, BusImpl::Signals::const_iterator end_)
    {
        _sid = BusImpl::create_new(name, begin_, end_);
    }

    std::size_t size() const { return _sid->size(); }
    BusImpl::OrderedKeys::const_iterator begin() const noexcept { return _sid->begin(); }
    BusImpl::OrderedKeys::const_iterator end() const noexcept { return _sid->end(); }
    const SignalImplPtr& operator[](std::size_t index) const { return (*_sid)[index]; }
    const SignalImplPtr& at(std::size_t index) const { return _sid->at(index); }
    const SignalImplPtr& operator[](const std::string& label) const { return (*_sid)[label]; }
    const SignalImplPtr& at(const std::string& label) const { return _sid->at(label); }

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
