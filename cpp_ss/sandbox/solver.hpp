
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "blocks.hpp"

using namespace Eigen;

namespace blocks
{

using SolverCallback = std::function<NodeValues(double, const NodeValues&)>;
using Solver         = std::function<NodeValues(SolverCallback, double, double, const NodeValues&)>;

NodeValues rk4   (SolverCallback callback, double t0, double t1, const NodeValues& x0);
NodeValues simple(SolverCallback callback, double t0, double t1, const NodeValues& x0);

}

#endif // __SOLVER_HPP__
