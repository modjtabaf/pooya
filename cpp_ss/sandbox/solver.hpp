
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "blocks.hpp"

using namespace Eigen;

namespace blocks
{

using SolverCallback = std::function<VectorXd(double, const VectorXd&)>;
using Solver         = std::function<VectorXd(SolverCallback, double, double, const VectorXd&)>;

VectorXd rk4   (SolverCallback callback, double t0, double t1, const VectorXd& x0);
VectorXd simple(SolverCallback callback, double t0, double t1, const VectorXd& x0);

}

#endif // __SOLVER_HPP__
