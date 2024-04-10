
#include "util.hpp"

namespace pooya::util
{

void pooya_throw_exception(const std::string& file, int line, const std::string& msg)
{
    std::stringstream full_msg;
    full_msg <<
        "\nPooya Exception:\n" <<
        "  " << file << ":" << std::to_string(line) << "\n"
        "  " << msg << "\n\n";
#ifndef POOYA_NDEBUG
    full_msg << pooya_trace_info_string();
#endif // POOYA_NDEBUG
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
            msg << "  " << it->_msg << "\n";
    }
    return msg.str();
}

#endif // !defined(POOYA_NDEBUG)

}
