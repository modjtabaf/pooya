
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "blocks.hpp"

using namespace Eigen;

namespace blocks
{

using SolverCallback = std::function<Signal(double, const Signal&)>;
using Solver         = std::function<Signal(SolverCallback, double, double, const Signal&)>;

Signal rk4   (SolverCallback callback, double t0, double t1, const Signal& x0);
Signal simple(SolverCallback callback, double t0, double t1, const Signal& x0);

}

#endif // __SOLVER_HPP__
