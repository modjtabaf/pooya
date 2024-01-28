
#include "util.hpp"

namespace pooya::util
{

#ifndef POOYA_NDEBUG

std::vector<PooyaTraceInfo> pooya_trace_info;

void pooya_throw_exception(const std::string& file, int line, const std::string& msg)
{
    std::stringstream full_msg;
    full_msg <<
        "\nPooya Exception:\n" <<
        "  " << file << ":" << std::to_string(line) << "\n"
        "  " << msg << "\n\n";
    full_msg << "Pooya Traceback:\n";
    for (auto it=pooya::util::pooya_trace_info.rbegin(); it != pooya::util::pooya_trace_info.rend(); it++)
    {
        full_msg <<
            "- " + it->_file << ":" << std::to_string(it->_line) << "\n";
        if (!it->_msg.empty())
            full_msg << "  " << it->_msg << "\n";
    }
    throw std::runtime_error(full_msg.str());
}

#endif // !defined(POOYA_NDEBUG)

}
