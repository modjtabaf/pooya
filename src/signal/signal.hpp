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
protected:
    struct Protected
    {
    };

    SignalImpl(const ValidName& name) : NamedObject(name) {}

public:
    virtual ~SignalImpl() = default;
};

class Signal
{
protected:
    std::shared_ptr<SignalImpl> _sid; // move this to Signal, make it a parent maybe?

public:
    explicit Signal(SignalImpl* sig) : _sid(sig ? sig->shared_from_this() : nullptr) {}

    Signal(const Signal&) = default;

    const SignalImpl* get() const { return _sid.get(); }

    SignalImpl* get() { return _sid.get(); }

    const SignalImpl* operator->() const
    {
        pooya_verify(_sid, "Illegal attempt to access a null signal.");
        return get();
    }

    SignalImpl* operator->()
    {
        pooya_verify(_sid, "Illegal attempt to access a null signal.");
        return get();
    }

    const SignalImpl& operator*() const
    {
        pooya_verify(_sid, "Illegal attempt to dereference a null signal.");
        return *_sid;
    }

    SignalImpl& operator*()
    {
        pooya_verify(_sid, "Illegal attempt to dereference a null signal.");
        return *_sid;
    }

    explicit operator const SignalImpl*() const { return get(); }

    explicit operator SignalImpl*() { return get(); }

    explicit operator bool() const noexcept { return _sid.operator bool(); }
};

template<typename T>
class SignalT : public Signal
{
    using Base = Signal;

public:
    explicit SignalT(SignalImpl* sig) : Base(sig) {} // todo: rename sig to sig or similar everywhere

    SignalT(const Signal& sig) : Base(sig) {}

    typename Types<T>::Signal& operator=(SignalImpl* sig)
    {
        _sid = sig ? std::dynamic_pointer_cast<typename Types<T>::SignalImpl>(sig->shared_from_this()) : nullptr;
        return *static_cast<typename Types<T>::Signal*>(this);
    }

    typename Types<T>::Signal& operator=(const Signal& sig)
    {
        _sid = sig ? std::dynamic_pointer_cast<typename Types<T>::SignalImpl>(sig._sid->shared_from_this()) : nullptr;
        return *static_cast<typename Types<T>::Signal*>(this);
    }

    const typename Types<T>::SignalImpl* get() const
    {
        pooya_verify(!_sid || dynamic_cast<const typename Types<T>::SignalImpl*>(_sid.get()),
                     "Illegal cast attempted.");
        return static_cast<const typename Types<T>::SignalImpl*>(_sid.get());
    }

    typename Types<T>::SignalImpl* get()
    {
        pooya_verify(!_sid || dynamic_cast<typename Types<T>::SignalImpl*>(_sid.get()), "Illegal cast attempted.");
        return static_cast<typename Types<T>::SignalImpl*>(_sid.get());
    }

    const typename Types<T>::SignalImpl* operator->() const
    {
        const auto* ret = get();
        pooya_verify(ret, "Illegal attempt to access a null signal.");
        return ret;
    }

    typename Types<T>::SignalImpl* operator->()
    {
        auto* ret = get();
        pooya_verify(ret, "Illegal attempt to access a null signal.");
        return ret;
    }

    operator const typename Types<T>::SignalImpl *() const { return get(); }

    operator typename Types<T>::SignalImpl *() { return get(); }

    const typename Types<T>::SignalImpl& operator*() const
    {
        const auto* ret = get();
        pooya_verify(ret, "Illegal attempt to dereference a null signal.");
        return *ret;
    }

    typename Types<T>::SignalImpl& operator*()
    {
        auto* ret = get();
        pooya_verify(ret, "Illegal attempt to dereference a null signal.");
        return *ret;
    }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_SIGNAL_HPP__
