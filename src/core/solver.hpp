
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "signal.hpp"

namespace pooya
{

using SolverCallback = std::function<void(double, Values&)>;
using Solver         = std::function<void(const Model&, SolverCallback, double, double, StatesInfo&, double&)>;

void euler(const Model& model, SolverCallback callback, double t0, double t1, StatesInfo& states, double& new_h);
void   rk4(const Model& model, SolverCallback callback, double t0, double t1, StatesInfo& states, double& new_h);
void rkf45(const Model& model, SolverCallback callback, double t0, double t1, StatesInfo& states, double& new_h);

}

#endif // __SOLVER_HPP__
