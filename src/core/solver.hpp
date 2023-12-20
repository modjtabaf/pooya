
#ifndef __SOLVER_HPP__
#define __SOLVER_HPP__

#include "signal.hpp"

namespace pooya
{

using SolverCallback = std::function<void(double, Values&)>;
using Solver         = std::function<const Array&(const Model&, SolverCallback, double, const Array&, double, double&)>;

const Array& euler(const Model& model, SolverCallback callback, double t0, const Array& v0, double t1, double& new_h);
const Array&   rk4(const Model& model, SolverCallback callback, double t0, const Array& v0, double t1, double& new_h);
const Array& rkf45(const Model& model, SolverCallback callback, double t0, const Array& v0, double t1, double& new_h);

}

#endif // __SOLVER_HPP__
