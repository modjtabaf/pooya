
#include <iostream>
#include <ostream>
#include <type_traits>

#include "blocks.hpp"
#include "solver.hpp"

namespace blocks
{

NodeValues rk4(SolverCallback callback, double t0, double t1, const NodeValues& x0)
{
    auto n = x0.first.size();
    double h = t1 - t0;

    // k1 = h*callback(t0, x0)
    Values values = callback(t0, x0);
    Values k1;
    k1.reserve(n);
    for (const auto& v: values)
        k1.push_back(h*v);

    // x1 = x0 + k1/2
    Values x1;
    x1.reserve(n);
    auto k1_it = k1.begin();
    for (const auto& v: x0.second)
        x1.push_back(v + *(k1_it++)/2);

    // k2 = h*callback(t0 + h/2, x1)
    values = callback(t0 + h/2, NodeValues(x0.first, x1));
    Values k2;
    k2.reserve(n);
    for (const auto& v: values)
        k2.push_back(h*v);

    // x2 = x0 + k2/2
    Values x2;
    x2.reserve(n);
    auto k2_it = k2.begin();
    for (const auto& v: x0.second)
        x2.push_back(v + *(k2_it++)/2);

    // k3 = h*callback(t0 + h/2, x2)
    values = callback(t0 + h/2, NodeValues(x0.first, x2));
    Values k3;
    k3.reserve(n);
    for (const auto& v: values)
        k3.push_back(h*v);

    // x3 = x0 + k3
    Values x3;
    x3.reserve(n);
    auto k3_it = k3.begin();
    for (const auto& v: x0.second)
        x3.push_back(v + *(k3_it++));

    // k4 = h*callback(t1, x3)
    values = callback(t1, NodeValues(x0.first, x3));
    Values k4;
    k4.reserve(n);
    for (const auto& v: values)
        k4.push_back(h*v);

    // ret = x0 + k1/6 + k2/3 + k3/3 + k4/6
    Values ret;
    ret.reserve(n);
    k1_it = k1.begin();
    k2_it = k2.begin();
    k3_it = k3.begin();
    auto k4_it = k4.begin();
    for (const auto& v: x0.second)
        ret.push_back(v + *(k1_it++)/6 + *(k2_it++)/3 + *(k3_it++)/3 + *(k4_it++)/6);

    return NodeValues(x0.first, ret);
}

NodeValues simple(SolverCallback callback, double t0, double t1, const NodeValues& x0)
{
    double h = t1 - t0;

    // ret = x0 + (t1 - t0)*callback(t0, x0)
    Values values = callback(t0, x0);
    Values ret;
    ret.reserve(values.size());
    auto x0_it = x0.second.begin();
    for (auto& v: values)
        ret.push_back(*(x0_it++) + h*v);

    return NodeValues(x0.first, ret);
}

}
