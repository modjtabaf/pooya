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

#include <memory>
#include <string>
#include <vector>

#include "signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"

#include <type_traits>
#include <utility>

template<typename T>
struct is_pair : std::false_type
{
};

template<typename T1, typename T2>
struct is_pair<std::pair<T1, T2>> : std::true_type
{
};

template<typename T>
constexpr bool is_pair_v = is_pair<T>::value;

namespace pooya
{

class BusSpec
{
};

class BusImpl : public SignalImpl
{
public:
    using LabelSignalImpl = std::pair<std::string, std::shared_ptr<SignalImpl>>;
    using Signals         = std::vector<LabelSignalImpl>;

protected:
    Signals _signals;

    static std::string _make_auto_label(std::size_t index) { return "sig" + std::to_string(index); }

public:
    template<typename T>
    BusImpl(Protected, std::initializer_list<T> l, const ValidName& name) : SignalImpl(name)
    {
        pooya_trace("name: " + name.str());

        _signals.reserve(l.size());

        int k{0};
        for (auto v : l)
        {
            if constexpr (is_pair_v<T>)
                _signals.emplace_back(ValidName(v.first).str(), v.second->shared_from_this());
            else
                _signals.emplace_back(_make_auto_label(k++), v->shared_from_this());
        }
    }

    template<typename T>
    static std::shared_ptr<BusImpl> create_new(std::initializer_list<T> l = {}, const ValidName& name = "")
    {
        pooya_trace("create_new: " + name.str());
        return std::make_shared<BusImpl>(Protected(), l, name);
    }

    std::size_t size() const { return _signals.size(); }
    Signals::const_iterator begin() const noexcept { return _signals.begin(); }
    Signals::const_iterator end() const noexcept { return _signals.end(); }

    SignalImpl& at(std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        auto* sig = _signals.at(index).second.get();
        pooya_verify(sig, "attempting to dereference a null signal.");
        return *sig;
    }

    template<typename T>
    typename Types<T>::SignalImpl& at(std::size_t index) const
    {
        auto* sig = dynamic_cast<typename Types<T>::SignalImpl*>(&at(index));
        pooya_verify(sig, "Invalid cast attempted.");
        return *sig;
    }

    SignalImpl& operator[](std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        pooya_verify(index < _signals.size(), "index out of range!");
        auto* sig = _signals[index].second.get();
        pooya_verify(sig, "attempting to dereference a null signal.");
        return *sig;
    }

    template<typename T>
    typename Types<T>::SignalImpl& operator[](std::size_t index) const
    {
        auto* sig = dynamic_cast<typename Types<T>::SignalImpl*>(&operator[](index));
        pooya_verify(sig, "Invalid cast attempted.");
        return *sig;
    }

    SignalImpl& at(std::string_view label) const
    {
        pooya_trace("label: " + std::string(label));

        auto pos                  = label.find(".");
        const auto& label_to_find = pos == std::string::npos ? label : label.substr(0, pos);
        const auto it             = std::find_if(_signals.begin(), _signals.end(),
                                                 [label_to_find](const auto& p) -> bool { return p.first == label_to_find; });
        if (it == _signals.end())
        {
            throw std::out_of_range("Label not found in the bus: " + std::string(label));
        }
        if (pos == std::string::npos)
        {
            pooya_verify(it->second.get(), "attempting to dereference a null signal.");
            return *it->second.get();
        }

        pooya_verify(dynamic_cast<BusImpl*>(it->second.get()), "Invalid cast attempted.");
        return static_cast<BusImpl*>(it->second.get())->at(label.substr(pos + 1));
    }

    template<typename T>
    typename Types<T>::SignalImpl& at(std::string_view label) const
    {
        auto* sig = dynamic_cast<typename Types<T>::SignalImpl*>(&at(label));
        pooya_verify(sig, "Invalid cast attempted.");
        return *sig;
    }

    SignalImpl& operator[](std::string_view label) const { return at(label); }

    template<typename T>
    typename Types<T>::SignalImpl& operator[](std::string_view label) const
    {
        return at<T>(label);
    }
};

class Bus : public SignalT<BusSpec>
{
    using Base = SignalT<BusSpec>;

public:
    // template <typename T=Signal>
    // Bus(std::initializer_list<T> l = {}) : Base(BusImpl::create_new("", l).get()) {}

    template<typename T = Signal>
    Bus(std::initializer_list<T> l = {}, const ValidName& name = "") : Base(BusImpl::create_new(l, name).get())
    {
    }

    // template <typename T=Signal>
    // Bus(std::initializer_list<std::pair<std::string, T>> l) : Base(BusImpl::create_new("", l).get()) {}

    template<typename T = Signal>
    Bus(std::initializer_list<std::pair<std::string, T>> l, const ValidName& name = "")
        : Base(BusImpl::create_new(l, name).get())
    {
    }

    Bus(SignalImpl* sig) : Base(dynamic_cast<BusImpl*>(sig)) {}
};

} // namespace pooya

#endif // __POOYA_SIGNAL_BUS_HPP__
