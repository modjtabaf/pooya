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

#include "src/helper/trace.hpp"
#include "label_signal.hpp"

namespace pooya
{

void LabelSignals::_init(LabelSignalIdList::const_iterator begin_, LabelSignalIdList::const_iterator end_)
{
    pooya_trace0;
    for (auto it=begin_; it != end_; it++) {push_back(*it);}
}

LabelSignals::LabelSignals(const std::initializer_list<SignalId>& il)
{
    pooya_trace0;
    LabelSignalIdList lsl;
    std::size_t index=0;
    for (const auto& sig: il) {lsl.push_back({_make_auto_label(index++), sig});}
    _init(lsl.begin(), lsl.end());
}

LabelSignals::LabelSignals(const std::initializer_list<LabelSignalId>& il)
{
    pooya_trace0;
    LabelSignalIdList lsl(il);
    _init(lsl.begin(), lsl.end());
}

}
