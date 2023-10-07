
#include <iostream>
#include <ostream>
#include <type_traits>

#include "pooya.hpp"
#include "solver.hpp"

namespace pooya
{

#define STEP(K, t, X) \
    callback(t, X); \
    Values K(states.size()); \
    k = 0; \
    for (auto& state: states) \
    { \
        const auto* dx = X.get(state.second.second); \
        if (!dx) \
        { \
            std::cout << "Error: not all state derivatives are present. Aborting...\n"; \
            return; \
        } \
        K.set(k++, h * (*dx)); \
    }

void rk4(SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_signals, double& new_h)
{
    double h = new_h = t1 - t0;
    int k;

    // X0

    Values X0(num_signals);
    for (const auto& state: states)
        X0.set(state.first, state.second.first);

    // K1 = h * f(t0, X0)

    STEP(K1, t0, X0)

    // X1 = X0 + K1/2

    Values X1(num_signals);
    k = 0;
    for (const auto& state: states)
        X1.set(state.first, *X0.get(state.first) + (*K1.get(k++)) / 2);

    // K2 = h * f(t0 + h/2, X1)

    STEP(K2, t0 + h/2, X1)

    // X2 = X0 + K2/2

    Values X2(num_signals);
    k = 0;
    for (const auto& state: states)
        X2.set(state.first, *X0.get(state.first) + (*K2.get(k++)) / 2);

    // K3 = h * f(t0 + h/2, X2)

    STEP(K3, t0 + h/2, X2)

    // X3 = X0 + K3

    Values X3(num_signals);
    k = 0;
    for (const auto& state: states)
        X3.set(state.first, *X0.get(state.first) + (*K3.get(k++)));

    // K4 = h * f(t0 + h, X3)

    STEP(K4, t0 + h, X3)

    // ret = X0 + K1/6 + K2/3 + K3/3 + K4/6

    k = 0;
    for (auto& state: states)
    {
        state.second.first = *X0.get(state.first) + (*K1.get(k)) / 6 + (*K2.get(k)) / 3 + (*K3.get(k)) / 3 + (*K4.get(k)) / 6;
        k++;
    }
}

// source: https://ece.uwaterloo.ca/~dwharder/NumericalAnalysis/14IVPs/rkf45/complete.html
void rkf45(SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_signals, double& new_h)
{
    double h = t1 - t0;
    uint k;

    // f := (t, y) -> -2*y + exp(-2*(t - 6)^2);
    // dsolve( {D(y)(t) = f( t, y(t) ), y(0) = 1} );

    // t[0] := 0.0;
    // y[0] := 1.0;
    // h := 0.1;
    // eps[abs] := 0.001;
    double eps_abs = 0.001;

    // for i from 1 do

    //     K[1] := f( t[i-1],             y[i-1] );

    // X0

    Values X0(num_signals);
    for (const auto& state: states)
        X0.set(state.first, state.second.first);

    // K1 = h * f(t0, X0)
    STEP(K1, t0, X0)

    //     K[2] := f( t[i-1] +   (1/4)*h, y[i-1] +   (1/4)*h*K[1] );

    // X1 = X0 + K1/4

    Values X1(num_signals);
    k = 0;
    for (const auto& state: states)
        X1.set(state.first, *X0.get(state.first) + 1./4 * (*K1.get(k++)));

    // K2 = h * f(t0 + h/4, X1)
    STEP(K2, t0 + h/4, X1)

    //     K[3] := f( t[i-1] +   (3/8)*h, y[i-1] +   (3/8)*h*(        1/4*K[1] +         3/4*K[2]) );

    // X2 = X0 + 3/8 * ( 1/4 * K1 + 3/4 * K2)

    Values X2(num_signals);
    k = 0;
    for (const auto& state: states)
    {
        X2.set(state.first, *X0.get(state.first) + (3./8 * 1./4) * (*K1.get(k)) + (3./8 * 3./4) * (*K2.get(k)));
        k++;
    }

    // K3 = h * f(t0 + 3/8 * h, X2)
    STEP(K3, t0 + 3./8 * h, X2)

    //     K[4] := f( t[i-1] + (12/13)*h, y[i-1] + (12/13)*h*(    161/169*K[1] -     600/169*K[2] +     608/169*K[3]) );

    // X3 = X0 + 12/13 * (161/169 * K1 - 600/169 * K2 + 608/169 * K3)

    Values X3(num_signals);
    k = 0;
    for (const auto& state: states)
    {
        X3.set(state.first, *X0.get(state.first) + (12./13 * 161./169) * (*K1.get(k)) - (12./13 * 600./169) * (*K2.get(k)) + (12./13 * 608./169) * (*K3.get(k)));
        k++;
    }

    // K4 = h * f(t0 + 12/13 * h, X2)
    STEP(K4, t0 + 12./13 * h, X3)

    //     K[5] := f( t[i-1] +         h, y[i-1] +         h*(  8341/4104*K[1] -  32832/4104*K[2] +  29440/4104*K[3] -   845/4104*K[4]) );

    // X4 = X0 + 8341/4104 * K1 - 32832/4104 * K2 + 29440/4104 * K3 - 845/4104 * K4

    Values X4(num_signals);
    k = 0;
    for (const auto& state: states)
    {
        X4.set(state.first, *X0.get(state.first) + (8341./4104) * (*K1.get(k)) - (32832./4104) * (*K2.get(k)) + (29440./4104) * (*K3.get(k)) - (845./4104) * (*K4.get(k)));
        k++;
    }

    // K5 = h * f(t0 + h, X4)
    STEP(K5, t1, X4)

    //     K[6] := f( t[i-1] +   (1/2)*h, y[i-1] +   (1/2)*h*(-6080/10260*K[1] + 41040/10260*K[2] - 28352/10260*K[3] + 9295/10260*K[4] - 5643/10260*K[5]) );

    // X5 = X0 + 1/2 * (-6080/10260 * K1 + 41040/10260 * K2 - 28352/10260 * K3 + 9295/10260 * K4 - 5643/10260 * K5)

    Values X5(num_signals);
    k = 0;
    for (const auto& state: states)
    {
        X5.set(state.first, *X0.get(state.first) - (1./2 * 6080./10260) * (*K1.get(k)) + (1./2 * 41040./10260) * (*K2.get(k)) - (1./2 * 28352./10260) * (*K3.get(k)) + (1./2 * 9295./10260) * (*K4.get(k)) - (1./2 * 5643./10260) * (*K5.get(k)));
        k++;
    }

    // K6 = h * f(t0 + 1/2 * h, X5)
    STEP(K6, t0 + 1./2 * h, X5)

    //     yt := y[i - 1] + h*(  2375*K[1] +  11264*K[3] +  10985*K[4] - 4104*K[5] )/20520;

    Values YT(states.size());
    k = 0;
    for (const auto& state: states)
    {
        YT.set(k, *X0.get(state.first) + (2375./20520) * (*K1.get(k)) + (11264./20520) * (*K3.get(k)) + (10985./20520) * (*K4.get(k)) - (4104./20520) * (*K5.get(k)));
        k++;
    }

    //     zt := y[i - 1] + h*( 33440*K[1] + 146432*K[3] + 142805*K[4] - 50787*K[5] + 10260*K[6] )/282150;

    Values ZT(states.size());
    k = 0;
    for (const auto& state: states)
    {
        ZT.set(k, *X0.get(state.first) + (33440./282150) * (*K1.get(k)) + (146432./282150) * (*K3.get(k)) + (142805./282150) * (*K4.get(k)) - (50787./282150) * (*K5.get(k)) + (10260./282150) * (*K6.get(k)));
        k++;
    }

    //     s := root[4]( (eps[abs]*h)/(2*abs(yt - zt)) );

    double max_abs = 0;
    for (k = 0; k < states.size(); k++)
        max_abs = ((*YT.get(k)) - (*ZT.get(k))).array().abs().max(max_abs)(0, 0);

    //     if s < 0.75 then
    //         h := max( h/2, 0.025 );
    //         continue;
    //     end if;

    double s4 = max_abs > 0 ? (eps_abs * h) / (2 * max_abs) : 1;

    if (s4 < 0.31640625) // 0.31640625 == 0.75 ^ 4
        new_h = h / 2;
    else if (s4 > 5.0625) // 5.0625 == 1.5 ^ 4
        new_h = 2 * h;
    else
        new_h = h;

    //     t[i] := t[i - 1] + h;
    //     y[i] := yt;

    //     if t[i] >= 10 then
    //         break;
    //     end if;

    //     if s > 1.5 then
    //         h := min( 2*h, 1.6 );
    //     end if;
    // end do:

    k = 0;
    for (auto& state: states)
        state.second.first = *YT.get(k++);

    // plots[pointplot]( [seq( [t[k], y[k]], k = 1..i )] );
}

void simple(SolverCallback callback, double t0, double t1, StatesInfo& states, std::size_t num_signals, double& new_h)
{
    double h = new_h = t1 - t0;

    // ret = x0 + (t1 - t0)*callback(t0, x0)

    Values values(num_signals);

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
