
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

double Deriv::operator()(double t, double u)
{
    if (_first_step)
    {
        step(t, u);
        _first_step = false;
    }
    double dt = t - _t0;
    return dt == 0 ? 0 : (u - _u0)/dt;
}

void Deriv::step(double t, double u)
{
    _t0 = t;
    _u0 = u;
}

double PT1::operator()(double t, double u)
{
    double dudt = Deriv::operator()(t, u);
    return _k*_tau*dudt*(std::exp((_t0 - t)/_tau) - 1) + _k*u;
}

}
