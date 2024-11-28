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

#ifndef __POOYA_SIGNAL_TRAIT_HPP__
#define __POOYA_SIGNAL_TRAIT_HPP__

#include <memory>

#include "array.hpp"
#include "src/helper/verify.hpp"

namespace pooya
{

class SignalImpl;
using SignalImplPtr = std::shared_ptr<SignalImpl>;

class ValueSignalImpl;
using ValueSignalImplPtr = std::shared_ptr<ValueSignalImpl>;

class FloatSignalImpl;
using FloatSignalImplPtr = std::shared_ptr<FloatSignalImpl>;

template<typename T>
struct Types
{
};

class ArraySignalImpl;
using ArraySignalImplPtr = std::shared_ptr<ArraySignalImpl>;
class ArraySignal;

template<>
struct Types<Array>
{
    using Signal = ArraySignal;
    using SignalImpl    = ArraySignalImpl;
    using SignalImplPtr = ArraySignalImplPtr;
    using GetValue      = const Array&;
    using SetValue      = const Array&;
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_array();
    }
    template<typename T>
    static SignalImpl& as_signal_info(const T& sig)
    {
        return sig->as_array();
    }
    template<typename T>
    static SignalImplPtr as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_array(), "Illegal attempt to dereference a non-array as an array.");
        return std::static_pointer_cast<SignalImpl>(sig->shared_from_this());
    }
};

class ScalarSignalImpl;
using ScalarSignalImplPtr = std::shared_ptr<ScalarSignalImpl>;
class ScalarSignal;

template<>
struct Types<double>
{
    using Signal = ScalarSignal;
    using SignalImpl    = ScalarSignalImpl;
    using SignalImplPtr = ScalarSignalImplPtr;
    using GetValue      = double;
    using SetValue      = double;
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_scalar();
    }
    template<typename T>
    static SignalImpl& as_signal_info(const T& sig)
    {
        return sig->as_scalar();
    }
    template<typename T>
    static SignalImplPtr as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_scalar(), "Illegal attempt to dereference a non-scalar as a scalar.");
        return std::static_pointer_cast<SignalImpl>(sig->shared_from_this());
    }
};

class IntSignalImpl;
using IntSignalImplPtr = std::shared_ptr<IntSignalImpl>;
class IntSignal;

template<>
struct Types<int>
{
    using Signal = IntSignal;
    using SignalImpl    = IntSignalImpl;
    using SignalImplPtr = IntSignalImplPtr;
    using GetValue      = int;
    using SetValue      = int;
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_int();
    }
    template<typename T>
    static SignalImpl& as_signal_info(const T& sig)
    {
        return sig->as_int();
    }
    template<typename T>
    static SignalImplPtr as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_int(), "Illegal attempt to dereference a non-int as an int.");
        return std::static_pointer_cast<SignalImpl>(sig->shared_from_this());
    }
};

class BoolSignalImpl;
using BoolSignalImplPtr = std::shared_ptr<BoolSignalImpl>;
class BoolSignal;

template<>
struct Types<bool>
{
    using Signal = BoolSignal;
    using SignalImpl    = BoolSignalImpl;
    using SignalImplPtr = BoolSignalImplPtr;
    using GetValue      = bool;
    using SetValue      = bool;
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_bool();
    }
    template<typename T>
    static SignalImpl& as_signal_info(const T& sig)
    {
        return sig->as_bool();
    }
    template<typename T>
    static SignalImplPtr as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_bool(), "Illegal attempt to dereference a non-bool as a bool.");
        return std::static_pointer_cast<SignalImpl>(sig->shared_from_this());
    }
};

class BusImpl;
using BusImplPtr = std::shared_ptr<BusImpl>;
class BusSpec;
class Bus;

template<>
struct Types<BusSpec>
{
    using Signal = Bus;
    using SignalImpl    = BusImpl;
    using SignalImplPtr = BusImplPtr;
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_bus();
    }
    template<typename T>
    static SignalImpl& as_signal_info(const T& sig)
    {
        return sig->as_bus();
    }
    template<typename T>
    static SignalImplPtr as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_bus(), "Illegal attempt to dereference a non-bus as a bus.");
        return std::static_pointer_cast<SignalImpl>(sig->shared_from_this());
    }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_TRAIT_HPP__
