
#include <iostream>
#include <cmath>

#include "util.hpp"

namespace pooya::util
{

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

void pooya_throw_exception(const std::string& file, int line, const std::string& msg)
{
    std::stringstream full_msg;
    full_msg <<
        "\nPooya Exception:\n" <<
        "  " << file << ":" << std::to_string(line) << "\n"
        "  " << msg << "\n\n";
    full_msg << pooya_trace_info_string();
    throw std::runtime_error(full_msg.str());
}

#endif // !defined(POOYA_NDEBUG)

double PT1::operator()(double t, double u)
{
    double U = Deriv::operator()(t, u);
    double T_C = _T/_C;
    double K_C = _K/_C;

    return std::exp((_t0 - t) / T_C)*(_y0 - K_C*_u0 + K_C*T_C*U) + K_C*(u - T_C*U);
}

void PT1::step(double t, double u, double y)
{
    Deriv::step(t, u);
    _y0 = y;
}

}
