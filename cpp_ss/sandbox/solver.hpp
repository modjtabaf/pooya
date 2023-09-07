
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "blocks.hpp"

using namespace Eigen;

namespace blocks
{

using SolverCallback = std::function<NodeIdValues(double, const NodeIdValues&)>;
using Solver         = std::function<NodeIdValues(SolverCallback, double, double, const NodeIdValues&)>;

NodeIdValues rk4   (SolverCallback callback, double t0, double t1, const NodeIdValues& x0);
NodeIdValues simple(SolverCallback callback, double t0, double t1, const NodeIdValues& x0);

}

#endif // __SOLVER_HPP__
