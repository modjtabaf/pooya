
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "blocks.hpp"

using namespace Eigen;

namespace blocks
{

using SolverCallback = std::function<void(double, Values&)>;
using Solver         = std::function<void(SolverCallback, double, double, StatesInfo&, std::size_t, double&)>;

void rk4   (SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_signals, double& new_h);
void rkf45 (SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_signals, double& new_h);
void simple(SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_signals, double& new_h);

}

#endif // __SOLVER_HPP__
