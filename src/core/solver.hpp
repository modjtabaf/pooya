
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "pooya.hpp"

namespace pooya
{

using SolverCallback = std::function<void(double, Values&)>;
using Solver         = std::function<void(SolverCallback, const SignalRegistry&, double, double, StatesInfo&, double&)>;

void euler(SolverCallback callback, const SignalRegistry& signal_registry, double t0, double t1, StatesInfo& states, double& new_h);
void   rk4(SolverCallback callback, const SignalRegistry& signal_registry, double t0, double t1, StatesInfo& states, double& new_h);
void rkf45(SolverCallback callback, const SignalRegistry& signal_registry, double t0, double t1, StatesInfo& states, double& new_h);

}

#endif // __SOLVER_HPP__
