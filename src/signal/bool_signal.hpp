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

#ifndef __POOYA_SIGNAL_BOOL_SIGNAL_HPP__
#define __POOYA_SIGNAL_BOOL_SIGNAL_HPP__

#include <optional>

#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

#define pooya_verify_bool_signal(sig)                                                                                  \
    pooya_verify_valid_signal(sig);                                                                                    \
    pooya_verify((sig)->is_bool(), (sig)->name().str() + ": bool signal expected!");

namespace pooya
{

class BoolSignalImpl : public ValueSignalImpl
{
protected:
    bool _bool_value;

public:
    BoolSignalImpl(Protected, const ValidName& name = "") : ValueSignalImpl(name, BoolType) {}

    static BoolSignalImplPtr create_new(const ValidName& name = "")
    {
        return std::make_shared<BoolSignalImpl>(Protected(), name);
    }

    bool get() const
    {
        pooya_trace0;
        pooya_verify(is_assigned(), name().str() + ": attempting to access an unassigned value!");
        return _bool_value;
    }

    void set(bool value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(!is_assigned(), name().str() + ": re-assignment is prohibited!");
        _bool_value = value;
        _assigned   = true;
    }

    operator bool() const { return get(); }
};

inline BoolSignalImpl& SignalImpl::as_bool()
{
    pooya_verify(_type & BoolType, "Illegal attempt to dereference a non-bool as a bool.");
    return *static_cast<BoolSignalImpl*>(this);
}

inline const BoolSignalImpl& SignalImpl::as_bool() const
{
    pooya_verify(_type & BoolType, "Illegal attempt to dereference a non-bool as a bool.");
    return *static_cast<const BoolSignalImpl*>(this);
}

class BoolSignal : public ValueSignal<bool>
{
    using Base = ValueSignal<bool>;

public:
    explicit BoolSignal(const ValidName& name = "") : Base(BoolSignalImpl::create_new(name)) {}
    explicit BoolSignal(const SignalImplPtr& sid)
        : Base(sid && sid->is_bool() ? std::static_pointer_cast<BoolSignalImpl>(sid) : nullptr)
    {
    }
    BoolSignal(const BoolSignalImplPtr& sid) : Base(sid) {}

    BoolSignal& operator=(const BoolSignal&) = delete;

    void reset(const ValidName& name = "") { _sid = BoolSignalImpl::create_new(name); }

    using ValueSignal<bool>::operator=;
    using Signal<bool>::reset;
};

} // namespace pooya

#endif // __POOYA_SIGNAL_BOOL_SIGNAL_HPP__
