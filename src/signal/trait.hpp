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

#ifndef __POOYA_SIGNAL_TRAIT_HPP__
#define __POOYA_SIGNAL_TRAIT_HPP__

#include "src/helper/verify.hpp"
#include "array.hpp"
#include "signal_id.hpp"

namespace pooya
{

class BusSpec;

template<typename T>
struct Types
{
};

template<>
struct Types<Array>
{
    using SignalInfo = ArraySignalInfo;
    using SignalId = ArraySignalId;
    using GetValue = const MappedArray&;
    using SetValue = const Array&;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
    static void verify_signal_type(pooya::SignalId sig, std::size_t size);
#endif // defined(POOYA_DEBUG)
    template<typename T> static SignalId as_type(const T& sig) {return sig->as_array();}
};

template<>
struct Types<double>
{
    using SignalInfo = ScalarSignalInfo;
    using SignalId = ScalarSignalId;
    using GetValue = double;
    using SetValue = double;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
#endif // defined(POOYA_DEBUG)
    template<typename T> static SignalId as_type(const T& sig) {return sig->as_scalar();}
};

template<>
struct Types<int>
{
    using SignalInfo = IntSignalInfo;
    using SignalId = IntSignalId;
    using GetValue = int;
    using SetValue = int;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
#endif // defined(POOYA_DEBUG)
    template<typename T> static SignalId as_type(const T& sig) {return sig->as_int();}
};

template<>
struct Types<bool>
{
    using SignalInfo = BoolSignalInfo;
    using SignalId = BoolSignalId;
    using GetValue = bool;
    using SetValue = bool;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig);
#endif // defined(POOYA_DEBUG)
    template<typename T> static SignalId as_type(const T& sig) {return sig->as_bool();}
};

template<>
struct Types<BusSpec>
{
    using SignalInfo = BusInfo;
    using SignalId = BusId;
#if defined(POOYA_DEBUG)
    static void verify_signal_type(pooya::SignalId sig, const BusSpec& spec);
#endif // defined(POOYA_DEBUG)
    template<typename T> static SignalId as_type(const T& sig) {return sig->as_bus();}
};

}

#endif // __POOYA_SIGNAL_TRAIT_HPP__
