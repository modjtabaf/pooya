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

#include "array.hpp"
#include "src/helper/defs.hpp"
#include "src/helper/verify.hpp"

namespace pooya
{

template<typename T>
struct Types
{
};

#ifdef POOYA_ARRAY_SIGNAL

class ArraySignalImpl;
class ArraySignal;

template<>
struct Types<Array>
{
    using Signal     = ArraySignal;
    using SignalImpl = ArraySignalImpl;
    using GetValue   = const Array&;
    using SetValue   = const Array&;
};

#endif // POOYA_ARRAY_SIGNAL

class ScalarSignalImpl;
class ScalarSignal;

template<>
struct Types<double>
{
    using Signal     = ScalarSignal;
    using SignalImpl = ScalarSignalImpl;
    using GetValue   = double;
    using SetValue   = double;
};

#ifdef POOYA_INT_SIGNAL

class IntSignalImpl;
class IntSignal;

template<>
struct Types<int>
{
    using Signal     = IntSignal;
    using SignalImpl = IntSignalImpl;
    using GetValue   = int;
    using SetValue   = int;
};

#endif // POOYA_INT_SIGNAL

#ifdef POOYA_BOOL_SIGNAL

class BoolSignalImpl;
class BoolSignal;

template<>
struct Types<bool>
{
    using Signal     = BoolSignal;
    using SignalImpl = BoolSignalImpl;
    using GetValue   = bool;
    using SetValue   = bool;
};

#endif // POOYA_BOOL_SIGNAL

class BusImpl;
class BusSpec;
class Bus;

template<>
struct Types<BusSpec>
{
    using Signal     = Bus;
    using SignalImpl = BusImpl;
};

} // namespace pooya

#endif // __POOYA_SIGNAL_TRAIT_HPP__
