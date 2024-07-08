/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "util.hpp"
#include "block_base.hpp"

namespace pooya::util
{

void pooya_throw_exception(const std::string& file, int line, const std::string& msg)
{
    std::stringstream full_msg;
    full_msg <<
        "\nPooya Exception:\n" <<
        "  " << file << ":" << std::to_string(line) << "\n"
        "  " << msg << "\n\n";
#if !defined(POOYA_NDEBUG)
    full_msg << pooya_trace_info_string();
#endif // !defined(POOYA_NDEBUG)
    throw std::runtime_error(full_msg.str());
}

void pooya_show_warning(const std::string& file, int line, const std::string& msg)
{
    std::cout <<
        "\nPooya Warning:\n" <<
        "  " << file << ":" << std::to_string(line) << "\n"
        "  " << msg << "\n\n";
}

#ifndef POOYA_NDEBUG

std::vector<PooyaTraceInfo> pooya_trace_info;

std::string pooya_trace_info_string()
{
    std::stringstream msg;
    msg << "Pooya Traceback:\n";
    for (auto it=pooya::util::pooya_trace_info.rbegin(); it != pooya::util::pooya_trace_info.rend(); it++)
    {
        msg <<
            "- " + it->_file << ":" << std::to_string(it->_line) << "\n";
        if (!it->_msg.empty())
            {msg << "  " << it->_msg << "\n";}
    }
    return msg.str();
}

std::string make_valid_given_name(const std::string& given_name, pooya::Parent* parent)
{
    pooya_trace(given_name);

    std::string ret;
    if (given_name.empty())
    {
        ret = "unnamed_block";
    }
    else
    {
        ret = given_name;
        std::replace(ret.begin(), ret.end(), ' ', '_');
        std::replace(ret.begin(), ret.end(), '~', '_');
        std::replace(ret.begin(), ret.end(), '/', '_');
    }

    if (parent)
    {
        uint n = 0;

        std::string foo{ret};
        auto pooya_verify_unique_name_cb = [&] (const Block& c, uint32_t level) -> bool
        {
            return (level == 0) || (c.given_name() != foo);
        };

        while (!parent->traverse(pooya_verify_unique_name_cb, 0, 1))
            {foo = ret + "_" + std::to_string(n++);}

        ret = foo;
    }

    return ret;
}

#endif // !defined(POOYA_NDEBUG)

}
