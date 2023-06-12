
#include <iostream>
#include <type_traits>

#include "solver.hpp"

namespace blocks
{

VectorXd rk4(SolverCallback callback, double t0, double t1, const VectorXd& x0)
{
    double h = t1 - t0;
    VectorXd k1 = h*callback(t0, x0);
    VectorXd k2 = h*callback(t0 + h/2, x0 + k1/2);
    VectorXd k3 = h*callback(t0 + h/2, x0 + k2/2);
    VectorXd k4 = h*callback(t1, x0 + k3);
    return x0 + k1/6 + k2/3 + k3/3 + k4/6;
}

VectorXd simple(SolverCallback callback, double t0, double t1, const VectorXd& x0)
{
    return x0 + (t1 - t0)*callback(t0, x0);
}

}
