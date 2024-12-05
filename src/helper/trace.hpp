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

#ifndef __POOYA_HELPER_TRACE_HPP__
#define __POOYA_HELPER_TRACE_HPP__

#include <string>
#include <vector>

#include "ndebug.hpp"
#include "verify.hpp"

namespace pooya::helper
{

#if !defined(POOYA_DEBUG)

#define pooya_trace(msg)
#define pooya_trace_update(msg)

#else

std::string pooya_trace_info_string();

struct PooyaTraceInfo
{
    std::string _file;
    int _line{-1};
    std::string _msg;
};

extern std::vector<PooyaTraceInfo> pooya_trace_info;

class PooyaTracer
{
public:
    PooyaTracer() { pooya_trace_info.emplace_back(); }

    static void update(const std::string& file, int line, const std::string& msg)
    {
        pooya_verify(!pooya_trace_info.empty(), "Empty trace queue!") auto& pt = pooya_trace_info.back();
        pt._file                                                               = file;
        pt._line                                                               = line;
        if (!msg.empty())
        {
            pt._msg = msg;
        }
    }

    ~PooyaTracer() { pooya_trace_info.pop_back(); }
};

#define pooya_trace(msg)                                                                                               \
    const pooya::helper::PooyaTracer __pooya_tracer__;                                                                 \
    pooya_trace_update(msg);
#define pooya_trace_update(msg) __pooya_tracer__.update(__FILE__, __LINE__, msg);

#endif // !defined(POOYA_DEBUG)

#define pooya_trace0 pooya_trace("")
#define pooya_trace_update0 pooya_trace_update("")

} // namespace pooya::helper

#endif // __POOYA_HELPER_TRACE_HPP__
