
#include <iostream>
#include <ostream>
#include <type_traits>

#include "blocks.hpp"
#include "solver.hpp"

namespace blocks
{

void rk4(SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_nodes)
{
    double h = t1 - t0;

    Values X0(num_nodes);
    for (const auto& state: states)
        X0.set(state.first, state.second.first);

    // K1 = h*callback(t0, X0)

    callback(t0, X0);

    Values K1(states.size());
    int k = 0;
    for (auto& state: states)
    {
        const auto* dx = X0.get(state.second.second);
        if (!dx)
        {
            std::cout << "Error: not all state derivatives are present. Aborting...\n";
            return;
        }
        K1.set(k++, h * (*dx));
    }

    // X1 = X0 + K1/2

    Values X1(num_nodes);
    k = 0;
    for (const auto& state: states)
        X1.set(state.first, *X0.get(state.first) + (*K1.get(k++)) / 2);

    // K2 = h*callback(t0 + h/2, X1)

    callback(t0 + h/2, X1);

    Values K2(states.size());
    k = 0;
    for (auto& state: states)
    {
        const auto* dx = X1.get(state.second.second);
        if (!dx)
        {
            std::cout << "Error: not all state derivatives are present. Aborting...\n";
            return;
        }
        K2.set(k++, h * (*dx));
    }

    // X2 = X0 + K2/2

    Values X2(num_nodes);
    k = 0;
    for (const auto& state: states)
        X2.set(state.first, *X0.get(state.first) + (*K2.get(k++)) / 2);

    // K3 = h*callback(t0 + h/2, X2)

    callback(t0 + h/2, X2);

    Values K3(states.size());
    k = 0;
    for (auto& state: states)
    {
        const auto* dx = X2.get(state.second.second);
        if (!dx)
        {
            std::cout << "Error: not all state derivatives are present. Aborting...\n";
            return;
        }
        K3.set(k++, h * (*dx));
    }

    // X3 = X0 + K3

    Values X3(num_nodes);
    k = 0;
    for (const auto& state: states)
        X3.set(state.first, *X0.get(state.first) + (*K3.get(k++)));

    // K4 = h*callback(t1, X3)

    callback(t1, X3);

    Values K4(states.size());
    k = 0;
    for (auto& state: states)
    {
        const auto* dx = X3.get(state.second.second);
        if (!dx)
        {
            std::cout << "Error: not all state derivatives are present. Aborting...\n";
            return;
        }
        K4.set(k++, h * (*dx));
    }

    // ret = X0 + K1/6 + K2/3 + K3/3 + K4/6

    k = 0;
    for (auto& state: states)
    {
        state.second.first = *X0.get(state.first) + (*K1.get(k)) / 6 + (*K2.get(k)) / 3 + (*K3.get(k)) / 3 + (*K4.get(k)) / 6;
        k++;
    }
}

void simple(SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_nodes)
{
    double h = t1 - t0;

    // ret = x0 + (t1 - t0)*callback(t0, x0)
    
    Values values(num_nodes);

    for (const auto& state: states)
        values.set(state.first, state.second.first);

    callback(t0, values);

    for (auto& state: states)
    {
        const auto* x = values.get(state.first);
        const auto* dx = values.get(state.second.second);
        if (!dx)
        {
            std::cout << "Error: not all state derivatives are present. Aborting...\n";
            return;
        }
        state.second.first = *x + h*(*dx);
    }
}

}
