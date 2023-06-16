
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "blocks.hpp"

using namespace Eigen;

namespace blocks
{

using SolverCallback = std::function<Value(double, const Value&)>;
using Solver         = std::function<Value(SolverCallback, double, double, const Value&)>;

Value rk4   (SolverCallback callback, double t0, double t1, const Value& x0);
Value simple(SolverCallback callback, double t0, double t1, const Value& x0);

}

#endif // __SOLVER_HPP__
