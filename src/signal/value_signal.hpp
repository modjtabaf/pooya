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

#ifndef __POOYA_SIGNAL_VALUE_SIGNAL_HPP__
#define __POOYA_SIGNAL_VALUE_SIGNAL_HPP__

#include "signal.hpp"

namespace pooya
{

class ValueSignalImpl : public SignalImpl
{
public:
    using Base = SignalImpl;
    using Ptr  = std::shared_ptr<ValueSignalImpl>;

    void clear() { _assigned = false; }
    bool is_assigned() const { return _assigned; }

protected:
    bool _assigned{false}; // has the value been assigned?

    ValueSignalImpl(std::string_view name) : SignalImpl(name) {}
};

} // namespace pooya

#endif // __POOYA_SIGNAL_VALUE_SIGNAL_HPP__
