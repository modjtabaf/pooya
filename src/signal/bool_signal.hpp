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

#ifdef POOYA_BOOL_SIGNAL

#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

namespace pooya
{

class BoolSignalImpl : public ValueSignalImpl
{
public:
    using Base = ValueSignalImpl;
    using Ptr  = std::shared_ptr<BoolSignalImpl>;

    BoolSignalImpl(Protected, const ValidName& name = "") : Base(name) {}

    static Ptr create_new(const ValidName& name = "") { return std::make_shared<BoolSignalImpl>(Protected(), name); }

    bool get_value() const
    {
        pooya_trace0;
        pooya_debug_verify(is_assigned(), name().str() + ": attempting to access an unassigned value!");
        return _bool_value;
    }

    void set_value(bool value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_debug_verify(!is_assigned(), name().str() + ": re-assignment is prohibited!");
        _bool_value = value;
        _assigned   = true;
    }

protected:
    bool _bool_value;
};

class BoolSignal : public SignalT<bool>
{
public:
    using Base = SignalT<bool>;

    BoolSignal() : BoolSignal("") {}
    BoolSignal(const BoolSignal& sig) : Base(sig) {}

    explicit BoolSignal(const ValidName& name) : Base(*BoolSignalImpl::create_new(name).get()) {}
    explicit BoolSignal(SignalImpl& sig) : Base(sig) {}
    explicit BoolSignal(const Signal& sig) : Base(sig) {}

    void operator=(const Signal&) = delete;
    void operator=(const BoolSignal& sig) { _typed_ptr->set_value(sig); }
    void operator=(bool value) { _typed_ptr->set_value(value); }

    operator bool() const { return _typed_ptr->get_value(); }
};

} // namespace pooya

#endif // POOYA_BOOL_SIGNAL

#endif // __POOYA_SIGNAL_BOOL_SIGNAL_HPP__
