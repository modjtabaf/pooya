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

#ifndef __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
#define __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__

#include "signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"
#include <memory>

#define  pooya_verify_float_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_float(), (sig)->name().str() + ": float signal expected!");

namespace pooya
{

class FloatSignalImpl : public ValueSignalImpl
{
protected:
    FloatSignalImplPtr _deriv_sig{nullptr}; // the derivative signal if this is a state variable, nullptr otherwise
    const std::size_t _size;

    FloatSignalImpl(const ValidName& name, uint32_t type, std::size_t size=1) :
        ValueSignalImpl(name, type | FloatType), _size(size) {}

public:
    bool is_state_variable() const {return static_cast<bool>(_deriv_sig);}
    std::size_t size() const {return _size;}
    void set_deriv_signal(FloatSignalImplPtr deriv_sig)
    {
        pooya_verify_valid_signal(deriv_sig);
        pooya_verify(_type == deriv_sig->_type, name().str() + ", " + deriv_sig->name().str() + ": type mismatch!");
        pooya_verify(_size == deriv_sig->_size, name().str() + ", " + deriv_sig->name().str() + ": size mismatch!");

        _deriv_sig = deriv_sig;
    }
    FloatSignalImplPtr& deriv_signal() {return _deriv_sig;}
    const FloatSignalImplPtr& deriv_signal() const {return _deriv_sig;}
};

inline FloatSignalImpl& SignalImpl::as_float()
{
    pooya_verify(_type & FloatType, "Illegal attempt to dereference a non-float as a float.");
    return *static_cast<FloatSignalImpl*>(this);
}

inline const FloatSignalImpl& SignalImpl::as_float() const
{
    pooya_verify(_type & FloatType, "Illegal attempt to dereference a non-float as a float.");
    return *static_cast<const FloatSignalImpl*>(this);
}

template<typename Derived, typename T>
class FloatSignal : public ValueSignal<Derived, T>
{
public:
    explicit FloatSignal(const typename Types<T>::SignalImplPtr& sid) : ValueSignal<Derived, T>(sid) {}

    FloatSignal& operator=(const FloatSignal&) = delete;

    void set_deriv_signal(FloatSignalImplPtr deriv_sig) {static_cast<Derived*>(this)->id()->set_deriv_signal(deriv_sig);}

    operator FloatSignalImplPtr() const {return std::static_pointer_cast<FloatSignalImpl>(static_cast<const Derived*>(this)->id()->shared_from_this());}
};

}

#endif // __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
