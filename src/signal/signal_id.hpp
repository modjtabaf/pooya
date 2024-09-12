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

#ifndef __POOYA_SIGNAL_SIGNAL_ID_HPP__
#define __POOYA_SIGNAL_SIGNAL_ID_HPP__

#if defined(POOYA_USE_SMART_PTRS)
#include <memory>
#endif // defined(POOYA_USE_SMART_PTRS)

namespace pooya
{

class       SignalInfo;
class  ValueSignalInfo;
class  FloatSignalInfo;
class ScalarSignalInfo;
class    IntSignalInfo;
class   BoolSignalInfo;
class  ArraySignalInfo;
class          BusInfo;

#if defined(POOYA_USE_SMART_PTRS)

using       SignalId =       std::shared_ptr<SignalInfo>;
using  ValueSignalId =  std::shared_ptr<ValueSignalInfo>;
using  FloatSignalId =  std::shared_ptr<FloatSignalInfo>;
using ScalarSignalId = std::shared_ptr<ScalarSignalInfo>;
using    IntSignalId =    std::shared_ptr<IntSignalInfo>;
using   BoolSignalId =   std::shared_ptr<BoolSignalInfo>;
using  ArraySignalId =  std::shared_ptr<ArraySignalInfo>;
using          BusId =          std::shared_ptr<BusInfo>;

using       ReadOnlySignalId = std::shared_ptr<const SignalInfo>;
using  ReadOnlyValueSignalId =  std::shared_ptr<const ValueSignalInfo>;
using  ReadOnlyFloatSignalId =  std::shared_ptr<const FloatSignalInfo>;
using ReadOnlyScalarSignalId = std::shared_ptr<const ScalarSignalInfo>;
using    ReadOnlyIntSignalId =    std::shared_ptr<const IntSignalInfo>;
using   ReadOnlyBoolSignalId =   std::shared_ptr<const BoolSignalInfo>;
using  ReadOnlyArraySignalId =  std::shared_ptr<const ArraySignalInfo>;
using          ReadOnlyBusId =          std::shared_ptr<const BusInfo>;

#else // defined(POOYA_USE_SMART_PTRS)

using       SignalId =       SignalInfo*;
using  ValueSignalId =  ValueSignalInfo*;
using  FloatSignalId =  FloatSignalInfo*;
using ScalarSignalId = ScalarSignalInfo*;
using    IntSignalId =    IntSignalInfo*;
using   BoolSignalId =   BoolSignalInfo*;
using  ArraySignalId =  ArraySignalInfo*;
using          BusId =          BusInfo*;

using       ReadOnlySignalId =       const SignalInfo*;
using  ReadOnlyValueSignalId =  const ValueSignalInfo*;
using  ReadOnlyFloatSignalId =  const FloatSignalInfo*;
using ReadOnlyScalarSignalId = const ScalarSignalInfo*;
using    ReadOnlyIntSignalId =    const IntSignalInfo*;
using   ReadOnlyBoolSignalId =   const BoolSignalInfo*;
using  ReadOnlyArraySignalId =  const ArraySignalInfo*;
using          ReadOnlyBusId =          const BusInfo*;

#endif // defined(POOYA_USE_SMART_PTRS)

}

#endif // __POOYA_SIGNAL_SIGNAL_ID_HPP__
