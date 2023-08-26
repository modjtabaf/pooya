
#include <iostream>
#include <ostream>
#include <type_traits>

#include "blocks.hpp"
#include "solver.hpp"

namespace blocks
{

NodeValues rk4(SolverCallback callback, double t0, double t1, const NodeValues& x0)
{
    // std::cout << "14: x0\n";
    // for (const auto& v: x0)
    //     std::cout << " - " << v.first << " = " << v.second << "\n";
    auto n = x0.size();
    double h = t1 - t0;

    Nodes nodes;
    nodes.reserve(n);
    for (const auto& v: x0)
        nodes.push_back(v.first);

    // k1 = h*callback(t0, x0)
    auto dx = callback(t0, x0);
    // std::cout << "27: xdot\n";
    // for (const auto& v: dx)
    //     std::cout << " - " << v << "\n";
    Values k1;
    k1.reserve(n);
    for (const auto& node: nodes)
        k1.push_back(h*dx.at(node));

    // x1 = x0 + k1/2
    Values x1;
    x1.reserve(n);
    auto k1_it = k1.begin();
    for (const auto& node: nodes)
        x1.push_back(x0.at(node) + *(k1_it++)/2);

    // k2 = h*callback(t0 + h/2, x1)
    dx = callback(t0 + h/2, NodeValues(nodes, x1));
    Values k2;
    k2.reserve(n);
    for (const auto& node: nodes)
        k2.push_back(h*dx.at(node));

    // x2 = x0 + k2/2
    Values x2;
    x2.reserve(n);
    auto k2_it = k2.begin();
    for (const auto& node: nodes)
        x2.push_back(x0.at(node) + *(k2_it++)/2);

    // k3 = h*callback(t0 + h/2, x2)
    dx = callback(t0 + h/2, NodeValues(nodes, x2));
    Values k3;
    k3.reserve(n);
    for (const auto& node: nodes)
        k3.push_back(h*dx.at(node));

    // x3 = x0 + k3
    Values x3;
    x3.reserve(n);
    auto k3_it = k3.begin();
    for (const auto& node: nodes)
        x3.push_back(x0.at(node) + *(k3_it++));

    // k4 = h*callback(t1, x3)
    dx = callback(t1, NodeValues(nodes, x3));
    Values k4;
    k4.reserve(n);
    for (const auto& node: nodes)
        k4.push_back(h*dx.at(node));

    // ret = x0 + k1/6 + k2/3 + k3/3 + k4/6
    Values ret;
    ret.reserve(n);
    k1_it = k1.begin();
    k2_it = k2.begin();
    k3_it = k3.begin();
    auto k4_it = k4.begin();
    for (const auto& node: nodes)
        ret.push_back(x0.at(node) + *(k1_it++)/6 + *(k2_it++)/3 + *(k3_it++)/3 + *(k4_it++)/6);

    return NodeValues(nodes, ret);
}

NodeValues simple(SolverCallback callback, double t0, double t1, const NodeValues& x0)
{
    auto n = x0.size();
    double h = t1 - t0;

    Nodes nodes;
    nodes.reserve(n);
    for (const auto& v: x0)
        nodes.push_back(v.first);

    // ret = x0 + (t1 - t0)*callback(t0, x0)
    auto dx = callback(t0, x0);
    Values ret;
    ret.reserve(dx.size());
    for (auto& node: nodes)
        ret.push_back(x0.at(node) + h*dx.at(node));

    return NodeValues(nodes, ret);
}

}
