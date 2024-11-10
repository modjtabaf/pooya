/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SIGNAL_TRAIT_HPP__
#define __POOYA_SIGNAL_TRAIT_HPP__

#include "array.hpp"
#include "signal_id.hpp"
#include "src/helper/verify.hpp"

namespace pooya
{

class BusSpec;

template<typename T>
struct Types
{
};

class ArraySignal;

template<>
struct Types<Array>
{
    using SignalWrapper = ArraySignal;
    using SignalInfo    = ArraySignalInfo;
    using SignalId      = ArraySignalId;
    using GetValue      = const Array&;
    using SetValue      = const Array&;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
    static void verify_signal_type(pooya::SignalId sig, std::size_t size);
#endif // defined(POOYA_DEBUG)
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_array();
    }
    template<typename T>
    static SignalInfo& as_signal_info(const T& sig)
    {
        return sig->as_array();
    }
    template<typename T>
    static SignalId as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_array(), "Illegal attempt to dereference a non-array as an array.");
        return std::static_pointer_cast<SignalInfo>(sig->shared_from_this());
    }
};

class ScalarSignal;

template<>
struct Types<double>
{
    using SignalWrapper = ScalarSignal;
    using SignalInfo    = ScalarSignalInfo;
    using SignalId      = ScalarSignalId;
    using GetValue      = double;
    using SetValue      = double;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
#endif // defined(POOYA_DEBUG)
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_scalar();
    }
    template<typename T>
    static SignalInfo& as_signal_info(const T& sig)
    {
        return sig->as_scalar();
    }
    template<typename T>
    static SignalId as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_scalar(), "Illegal attempt to dereference a non-scalar as a scalar.");
        return std::static_pointer_cast<SignalInfo>(sig->shared_from_this());
    }
};

class IntSignal;

template<>
struct Types<int>
{
    using SignalWrapper = IntSignal;
    using SignalInfo    = IntSignalInfo;
    using SignalId      = IntSignalId;
    using GetValue      = int;
    using SetValue      = int;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
#endif // defined(POOYA_DEBUG)
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_int();
    }
    template<typename T>
    static SignalInfo& as_signal_info(const T& sig)
    {
        return sig->as_int();
    }
    template<typename T>
    static SignalId as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_int(), "Illegal attempt to dereference a non-int as an int.");
        return std::static_pointer_cast<SignalInfo>(sig->shared_from_this());
    }
};

class BoolSignal;

template<>
struct Types<bool>
{
    using SignalWrapper = BoolSignal;
    using SignalInfo    = BoolSignalInfo;
    using SignalId      = BoolSignalId;
    using GetValue      = bool;
    using SetValue      = bool;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
#endif // defined(POOYA_DEBUG)
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_bool();
    }
    template<typename T>
    static SignalInfo& as_signal_info(const T& sig)
    {
        return sig->as_bool();
    }
    template<typename T>
    static SignalId as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_bool(), "Illegal attempt to dereference a non-bool as a bool.");
        return std::static_pointer_cast<SignalInfo>(sig->shared_from_this());
    }
};

template<>
struct Types<BusSpec>
{
    using SignalInfo = BusInfo;
    using SignalId   = BusId;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig, const BusSpec& spec);
#endif // defined(POOYA_DEBUG)
    template<typename T>
    static bool is_same_type(const T& sig)
    {
        return sig->is_bus();
    }
    template<typename T>
    static SignalInfo& as_signal_info(const T& sig)
    {
        return sig->as_bus();
    }
    template<typename T>
    static SignalId as_signal_id(const T& sig)
    {
        pooya_verify(sig->is_bus(), "Illegal attempt to dereference a non-bus as a bus.");
        return std::static_pointer_cast<SignalInfo>(sig->shared_from_this());
    }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_TRAIT_HPP__
