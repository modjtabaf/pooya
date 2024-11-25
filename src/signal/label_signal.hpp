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

#ifndef __POOYA_SIGNAL_LABEL_SIGNAL_HPP__
#define __POOYA_SIGNAL_LABEL_SIGNAL_HPP__

#include <map>
#include <string>
#include <vector>

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "signal.hpp"

namespace pooya
{

using LabelSignalImplPtr = std::pair<std::string, SignalImplPtr>;
using LabelSignalImplPtrList = std::vector<LabelSignalImplPtr>;
using LabelSignalImplPtrMap = std::map<LabelSignalImplPtr::first_type, LabelSignalImplPtr::second_type>;

class LabelSignals
{
protected:
    LabelSignalImplPtrMap _label_signals_map;
    LabelSignalImplPtrList _label_signals_list;

    std::string _make_auto_label(std::size_t index) const {return "sig" + std::to_string(index);}

    void _init(LabelSignalImplPtrList::const_iterator begin_, LabelSignalImplPtrList::const_iterator end_);

public:
    LabelSignals() = default;
    template<typename T>
    LabelSignals(const T& signal)
    {
        pooya_trace0;
        if constexpr(std::is_pointer_v<T>)
        {
            LabelSignalImplPtrList lsl({{_make_auto_label(0), signal->shared_from_this()}});
            _init(lsl.begin(), lsl.end());
        }
        else
        {
            LabelSignalImplPtrList lsl({{_make_auto_label(0), signal.id()->shared_from_this()}});
            _init(lsl.begin(), lsl.end());
        }
    }
    template<typename T>
    LabelSignals(const std::shared_ptr<T>& signal)
    {
        pooya_trace0;
        LabelSignalImplPtrList lsl({{_make_auto_label(0), signal->shared_from_this()}});
        _init(lsl.begin(), lsl.end());
    }
    LabelSignals(const std::initializer_list<SignalImplPtr>& il);
    LabelSignals(const std::initializer_list<LabelSignalImplPtr>& il);

    using const_iterator = LabelSignalImplPtrList::const_iterator;

    SignalImplPtr operator[](std::size_t index) const {return _label_signals_list[index].second;}
    SignalImplPtr operator[](const std::string& label) const {return _label_signals_map.at(label);}
    std::size_t size() const noexcept
    {
        return _label_signals_list.size();
    }
    const_iterator begin() const noexcept {return _label_signals_list.begin();}
    const_iterator end() const noexcept {return _label_signals_list.end();}
    bool push_back(const LabelSignalImplPtr& ls)
    {
        pooya_trace0;
        pooya_verify(!ls.first.empty(), "SignalImplPtr label can not be empty!");
        pooya_verify(ls.second, ls.first + ": invalid signal!");
        if (ls.first.empty() || (ls.second == nullptr)) {return false;}

        auto it = _label_signals_map.find(ls.first);
        pooya_verify(it == _label_signals_map.end(), ls.first + ": label already exists!");
        if (it != _label_signals_map.end()) {return false;}

        _label_signals_map[ls.first] = ls.second;
        _label_signals_list.push_back(ls);
        return true;
    }
};

std::ostream& operator<<(std::ostream& os, const LabelSignals& signals);

}

#endif // __POOYA_SIGNAL_LABEL_SIGNAL_HPP__
