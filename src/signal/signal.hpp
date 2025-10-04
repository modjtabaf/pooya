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

#ifndef __POOYA_SIGNAL_SIGNAL_HPP__
#define __POOYA_SIGNAL_SIGNAL_HPP__

#include <memory>

#include "src/helper/defs.hpp"
#include "src/helper/util.hpp"
#include "src/shared/named_object.hpp"
#include "trait.hpp"

namespace pooya
{

class SignalImpl : public std::enable_shared_from_this<SignalImpl>, public NamedObject
{
public:
    virtual ~SignalImpl() = default;

protected:
    struct Protected
    {
    };

    SignalImpl(std::string_view name) : NamedObject(name) {}
};

class Signal
{
public:
    template<typename T>
    static void fail_if_invalid_signal_type(SignalImpl* sig)
    {
        pooya_verify(dynamic_cast<typename Types<T>::SignalImpl*>(sig),
                     (sig ? std::string("(null)") : sig->name().str()) + ": invalid signal type!");
    }

    explicit Signal(SignalImpl& sig) : _ptr(std::move(sig.shared_from_this())) {}

    Signal(const Signal&) = default;

    SignalImpl& impl() const { return *_ptr; }
    SignalImpl& operator*() const { return *_ptr; }
    SignalImpl* operator->() const { return _ptr.get(); }

protected:
    std::shared_ptr<SignalImpl> _ptr;
};

template<typename T>
class SignalT : public Signal
{
public:
    using Base = Signal;
    using Impl = typename Types<T>::SignalImpl;

    explicit SignalT(SignalImpl& sig) : Base(sig)
    {
        fail_if_invalid_signal_type<T>(&sig);
        update_typed_ptr();
    }

    explicit SignalT(const Signal& sig) : Base(sig)
    {
        fail_if_invalid_signal_type<T>(&sig.impl());
        update_typed_ptr();
    }

    virtual void reset(const Signal& sig)
    {
        _ptr = std::dynamic_pointer_cast<Impl>(sig->shared_from_this());
        fail_if_invalid_signal_type<T>(_ptr.get());
        update_typed_ptr();
    }

    void operator=(const Signal&) = delete;

    Impl& impl() const { return *_typed_ptr.get(); }
    Impl* operator->() const { return _typed_ptr.get(); }

protected:
    std::shared_ptr<Impl> _typed_ptr;

    void update_typed_ptr()
    {
        _typed_ptr = std::dynamic_pointer_cast<Impl>(_ptr->shared_from_this());
        pooya_verify(_typed_ptr, "Illegal cast attempted.");
    }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_SIGNAL_HPP__
