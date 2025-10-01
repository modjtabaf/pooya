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
#include "src/helper/defs.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"

#include <type_traits>
#include <utility>

namespace pooya
{

class BusSpec
{
};

class BusImpl : public SignalImpl
{
public:
    using LabelSignalImpl = std::pair<std::string, Signal>;
    using Signals         = std::vector<LabelSignalImpl>;

    template<typename T>
    BusImpl(Protected, std::initializer_list<T> l, const ValidName& name) : SignalImpl(name)
    {
        pooya_trace("name: " + name.str());

        _signals.reserve(l.size());

        int k{0};
        for (auto v : l)
        {
            if constexpr (is_pair_v<T>)
            {
                const ValidName label(v.first);
                if (v.second.impl().name().str().empty()) v.second.impl().rename(label.str());
                _signals.emplace_back(label.str(), v.second.impl());
            }
            else
            {
                const ValidName label(_make_auto_label(k++));
                if (v.impl().name().str().empty()) v.impl().rename(label.str());
                _signals.emplace_back(label.str(), v.impl());
            }
        }
    }

    template<typename T>
    static std::shared_ptr<BusImpl> create_new(std::initializer_list<T> l = {}, const ValidName& name = "")
    {
        return std::make_shared<BusImpl>(Protected(), l, name);
    }

    std::size_t size() const { return _signals.size(); }
    Signals::const_iterator begin() const noexcept { return _signals.begin(); }
    Signals::const_iterator end() const noexcept { return _signals.end(); }

    Signal at(std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        return _signals.at(index).second;
    }

    Signal operator[](std::size_t index) const
    {
        pooya_trace("index: " + std::to_string(index));
        pooya_debug_verify(index < _signals.size(), "index out of range!");
        return _signals[index].second;
    }

    Signal at(std::string_view label) const
    {
        pooya_trace("label: " + std::string(label));

        auto pos                  = label.find(".");
        const auto& label_to_find = pos == std::string::npos ? label : label.substr(0, pos);
        const auto it             = std::find_if(_signals.begin(), _signals.end(),
                                                 [label_to_find](const auto& p) -> bool { return p.first == label_to_find; });
        pooya_verify(it != _signals.end(), "Label not found in the bus: " + std::string(label));

        if (pos == std::string::npos)
        {
            return it->second;
        }

        Signal::fail_if_invalid_signal_type<BusSpec>(&it->second.impl());
        BusImpl* bus = static_cast<BusImpl*>(&it->second.impl());
        return bus->at(label.substr(pos + 1));
    }

    std::optional<Signal> try_at(std::string_view label) const
    {
        pooya_trace("label: " + std::string(label));

        auto pos                  = label.find(".");
        const auto& label_to_find = pos == std::string::npos ? label : label.substr(0, pos);
        const auto it             = std::find_if(_signals.begin(), _signals.end(),
                                                 [label_to_find](const auto& p) -> bool { return p.first == label_to_find; });
        if (it == _signals.end())
        {
            return std::nullopt;
        }
        if (pos == std::string::npos)
        {
            return it->second;
        }

        BusImpl* bus = static_cast<BusImpl*>(&it->second.impl());
        if (!bus) return std::nullopt;
        return bus->at(label.substr(pos + 1));
    }

    Signal operator[](std::string_view label) const { return at(label); }

protected:
    Signals _signals;

    static std::string _make_auto_label(std::size_t index) { return "sig" + std::to_string(index); }
};

class Bus : public SignalT<BusSpec>
{
public:
    using Base = SignalT<BusSpec>;

    Bus(std::initializer_list<Signal> l = {}, const ValidName& name = "") : Base(*BusImpl::create_new(l, name)) {}

    template<typename T = Signal>
    Bus(std::initializer_list<std::pair<std::string, T>> l, const ValidName& name = "")
        : Base(*BusImpl::create_new(l, name))
    {
    }

    explicit Bus(SignalImpl& sig) : Base(sig) {}
    explicit Bus(const Signal& sig) : Base(sig) {}

    std::size_t size() const { return _typed_ptr->size(); }
    BusImpl::Signals::const_iterator begin() const noexcept { return _typed_ptr->begin(); }
    BusImpl::Signals::const_iterator end() const noexcept { return _typed_ptr->end(); }
    Signal at(std::size_t index) const { return _typed_ptr->at(index); }
    Signal operator[](std::size_t index) const { return _typed_ptr->operator[](index); }
    Signal at(std::string_view label) const { return _typed_ptr->at(label); }
    Signal operator[](std::string_view label) const { return _typed_ptr->operator[](label); }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_BUS_HPP__
