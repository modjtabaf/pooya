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

#include <sstream>

#include "trace.hpp"

#if defined(POOYA_DEBUG)

namespace pooya::helper
{

std::vector<PooyaTraceInfo> pooya_trace_info;

std::string pooya_trace_info_string()
{
    std::stringstream msg;
    msg << "Pooya Traceback:\n";
    for (auto it=pooya_trace_info.rbegin(); it != pooya_trace_info.rend(); it++)
    {
        msg <<
            "- " + it->_file << ":" << std::to_string(it->_line) << "\n";
        if (!it->_msg.empty())
            {msg << "  " << it->_msg << "\n";}
    }
    return msg.str();
}

}

#endif // defined(POOYA_DEBUG)
