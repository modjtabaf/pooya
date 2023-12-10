
#include <iostream>
#include <ostream>
#include <type_traits>

#include "pooya.hpp"
#include "solver.hpp"

namespace pooya
{

#define STEP(K, t, X) \
    callback(t, X); \
    static Values K(states); \
    K.invalidate(); \
    k = 0; \
    for (auto& state: states) \
    { \
        if (state._scalar) \
            K.set_scalar(k++, h * X.get_scalar(state._deriv_id)); \
        else \
            K.set_array(k++, h * X.get_array(state._deriv_id)); \
    }

void rk4(SolverCallback callback, const SignalRegistry& signal_registry, double t0, double t1, StatesInfo& states, double& new_h)
{
    double h = new_h = t1 - t0;
    int k;

    // X0

    static Values X0(signal_registry);
    X0.invalidate();
    for (const auto& state: states)
        if (state._scalar)
            X0.set_scalar(state._id, state._value[0]);
        else
            X0.set_array(state._id, state._value);

    // K1 = h * f(t0, X0)

    STEP(K1, t0, X0)

    // X1 = X0 + K1/2

    static Values X1(signal_registry);
    X1.invalidate();
    k = 0;
    for (const auto& state: states)
        if (state._scalar)
            X1.set_scalar(state._id, X0.get_scalar(state._id) + K1.get_scalar(k++) / 2);
        else
            X1.set_array(state._id, X0.get_array(state._id) + K1.get_array(k++) / 2);

    // K2 = h * f(t0 + h/2, X1)

    STEP(K2, t0 + h/2, X1)

    // X2 = X0 + K2/2

    static Values X2(signal_registry);
    X2.invalidate();
    k = 0;
    for (const auto& state: states)
        if (state._scalar)
            X2.set_scalar(state._id, X0.get_scalar(state._id) + K2.get_scalar(k++) / 2);
        else
            X2.set_array(state._id, X0.get_array(state._id) + K2.get_array(k++) / 2);

    // K3 = h * f(t0 + h/2, X2)

    STEP(K3, t0 + h/2, X2)

    // X3 = X0 + K3

    static Values X3(signal_registry);
    X3.invalidate();
    k = 0;
    for (const auto& state: states)
        if (state._scalar)
            X3.set_scalar(state._id, X0.get_scalar(state._id) + K3.get_scalar(k++));
        else
            X3.set_array(state._id, X0.get_array(state._id) + K3.get_array(k++));

    // K4 = h * f(t0 + h, X3)

    STEP(K4, t0 + h, X3)

    // ret = X0 + K1/6 + K2/3 + K3/3 + K4/6

    k = 0;
    for (auto& state: states)
    {
        if (state._scalar)
            state._value = X0.get_scalar(state._id) + K1.get_scalar(k) / 6 + K2.get_scalar(k) / 3 + K3.get_scalar(k) / 3 + K4.get_scalar(k) / 6;
        else
            state._value = X0.get_array(state._id) + K1.get_array(k) / 6 + K2.get_array(k) / 3 + K3.get_array(k) / 3 + K4.get_array(k) / 6;
        k++;
    }
}

// source: https://ece.uwaterloo.ca/~dwharder/NumericalAnalysis/14IVPs/rkf45/complete.html
void rkf45(SolverCallback callback, const SignalRegistry& signal_registry, double t0, double t1, StatesInfo& states, double& new_h)
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

    static Values X0(signal_registry);
    X0.invalidate();
    for (const auto& state: states)
        if (state._scalar)
            X0.set_scalar(state._id, state._value[0]);
        else
            X0.set_array(state._id, state._value);

    // K1 = h * f(t0, X0)
    STEP(K1, t0, X0)

    //     K[2] := f( t[i-1] +   (1/4)*h, y[i-1] +   (1/4)*h*K[1] );

    // X1 = X0 + K1/4

    static Values X1(signal_registry);
    X1.invalidate();
    k = 0;
    for (const auto& state: states)
        if (state._scalar)
            X1.set_scalar(state._id, X0.get_scalar(state._id) + 1./4 * K1.get_scalar(k++));
        else
            X1.set_array(state._id, X0.get_array(state._id) + 1./4 * K1.get_array(k++));

    // K2 = h * f(t0 + h/4, X1)
    STEP(K2, t0 + h/4, X1)

    //     K[3] := f( t[i-1] +   (3/8)*h, y[i-1] +   (3/8)*h*(        1/4*K[1] +         3/4*K[2]) );

    // X2 = X0 + 3/8 * ( 1/4 * K1 + 3/4 * K2)

    static Values X2(signal_registry);
    X2.invalidate();
    k = 0;
    for (const auto& state: states)
    {
        if (state._scalar)
            X2.set_scalar(state._id, X0.get_scalar(state._id) + (3./8 * 1./4) * K1.get_scalar(k) + (3./8 * 3./4) * K2.get_scalar(k));
        else
            X2.set_array(state._id, X0.get_array(state._id) + (3./8 * 1./4) * K1.get_array(k) + (3./8 * 3./4) * K2.get_array(k));
        k++;
    }

    // K3 = h * f(t0 + 3/8 * h, X2)
    STEP(K3, t0 + 3./8 * h, X2)

    //     K[4] := f( t[i-1] + (12/13)*h, y[i-1] + (12/13)*h*(    161/169*K[1] -     600/169*K[2] +     608/169*K[3]) );

    // X3 = X0 + 12/13 * (161/169 * K1 - 600/169 * K2 + 608/169 * K3)

    static Values X3(signal_registry);
    X3.invalidate();
    k = 0;
    for (const auto& state: states)
    {
        if (state._scalar)
            X3.set_scalar(state._id, X0.get_scalar(state._id) + (12./13 * 161./169) * K1.get_scalar(k) - (12./13 * 600./169) * K2.get_scalar(k) + (12./13 * 608./169) * K3.get_scalar(k));
        else
            X3.set_array(state._id, X0.get_array(state._id) + (12./13 * 161./169) * K1.get_array(k) - (12./13 * 600./169) * K2.get_array(k) + (12./13 * 608./169) * K3.get_array(k));
        k++;
    }

    // K4 = h * f(t0 + 12/13 * h, X2)
    STEP(K4, t0 + 12./13 * h, X3)

    //     K[5] := f( t[i-1] +         h, y[i-1] +         h*(  8341/4104*K[1] -  32832/4104*K[2] +  29440/4104*K[3] -   845/4104*K[4]) );

    // X4 = X0 + 8341/4104 * K1 - 32832/4104 * K2 + 29440/4104 * K3 - 845/4104 * K4

    static Values X4(signal_registry);
    X4.invalidate();
    k = 0;
    for (const auto& state: states)
    {
        if (state._scalar)
            X4.set_scalar(state._id, X0.get_scalar(state._id) + (8341./4104) * K1.get_scalar(k) - (32832./4104) * K2.get_scalar(k) + (29440./4104) * K3.get_scalar(k) - (845./4104) * K4.get_scalar(k));
        else
            X4.set_array(state._id, X0.get_array(state._id) + (8341./4104) * K1.get_array(k) - (32832./4104) * K2.get_array(k) + (29440./4104) * K3.get_array(k) - (845./4104) * K4.get_array(k));
        k++;
    }

    // K5 = h * f(t0 + h, X4)
    STEP(K5, t1, X4)

    //     K[6] := f( t[i-1] +   (1/2)*h, y[i-1] +   (1/2)*h*(-6080/10260*K[1] + 41040/10260*K[2] - 28352/10260*K[3] + 9295/10260*K[4] - 5643/10260*K[5]) );

    // X5 = X0 + 1/2 * (-6080/10260 * K1 + 41040/10260 * K2 - 28352/10260 * K3 + 9295/10260 * K4 - 5643/10260 * K5)

    static Values X5(signal_registry);
    X5.invalidate();
    k = 0;
    for (const auto& state: states)
    {
        if (state._scalar)
            X5.set_scalar(state._id, X0.get_scalar(state._id) - (1./2 * 6080./10260) * K1.get_scalar(k) + (1./2 * 41040./10260) * K2.get_scalar(k) - (1./2 * 28352./10260) * K3.get_scalar(k) + (1./2 * 9295./10260) * K4.get_scalar(k) - (1./2 * 5643./10260) * K5.get_scalar(k));
        else
            X5.set_array(state._id, X0.get_array(state._id) - (1./2 * 6080./10260) * K1.get_array(k) + (1./2 * 41040./10260) * K2.get_array(k) - (1./2 * 28352./10260) * K3.get_array(k) + (1./2 * 9295./10260) * K4.get_array(k) - (1./2 * 5643./10260) * K5.get_array(k));
        k++;
    }

    // K6 = h * f(t0 + 1/2 * h, X5)
    STEP(K6, t0 + 1./2 * h, X5)

    //     yt := y[i - 1] + h*(  2375*K[1] +  11264*K[3] +  10985*K[4] - 4104*K[5] )/20520;

    static Values YT(signal_registry);
    YT.invalidate();
    k = 0;
    for (const auto& state: states)
    {
        if (state._scalar)
            YT.set_scalar(k, X0.get_scalar(state._id) + (2375./20520) * K1.get_scalar(k) + (11264./20520) * K3.get_scalar(k) + (10985./20520) * K4.get_scalar(k) - (4104./20520) * K5.get_scalar(k));
        else
            YT.set_array(k, X0.get_array(state._id) + (2375./20520) * K1.get_array(k) + (11264./20520) * K3.get_array(k) + (10985./20520) * K4.get_array(k) - (4104./20520) * K5.get_array(k));
        k++;
    }

    //     zt := y[i - 1] + h*( 33440*K[1] + 146432*K[3] + 142805*K[4] - 50787*K[5] + 10260*K[6] )/282150;

    static Values ZT(signal_registry);
    ZT.invalidate();
    k = 0;
    for (const auto& state: states)
    {
        if (state._scalar)
            ZT.set_scalar(k, X0.get_scalar(state._id) + (33440./282150) * K1.get_scalar(k) + (146432./282150) * K3.get_scalar(k) + (142805./282150) * K4.get_scalar(k) - (50787./282150) * K5.get_scalar(k) + (10260./282150) * K6.get_scalar(k));
        else
            ZT.set_array(k, X0.get_array(state._id) + (33440./282150) * K1.get_array(k) + (146432./282150) * K3.get_array(k) + (142805./282150) * K4.get_array(k) - (50787./282150) * K5.get_array(k) + (10260./282150) * K6.get_array(k));
        k++;
    }

    //     s := root[4]( (eps[abs]*h)/(2*abs(yt - zt)) );

    double max_abs = 0;
    k = 0;
    for (const auto& state: states)
    {
        // max_abs = ((*YT.get_scalar(k)) - (*ZT.get_scalar(k))).array().abs().max(max_abs)(0, 0);
        if (state._scalar)
            max_abs = std::max(max_abs, std::abs(YT.get_scalar(k) - ZT.get_scalar(k)));
        else
            max_abs = (YT.get_array(k) - ZT.get_array(k)).abs().max(max_abs)(0, 0);
        k++;
    }

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
        if (state._scalar)
            state._value[0] = YT.get_scalar(k++);
        else
            state._value = YT.get_array(k++);

    // plots[pointplot]( [seq( [t[k], y[k]], k = 1..i )] );
}

void simple(SolverCallback callback, const SignalRegistry& signal_registry, double t0, double t1, StatesInfo& states, double& new_h)
{
    double h = new_h = t1 - t0;

    // ret = x0 + (t1 - t0)*callback(t0, x0)

    static Values values(signal_registry);
    values.invalidate();

    for (const auto& state: states)
        if (state._scalar)
            values.set_scalar(state._id, state._value[0]);
        else
            values.set_array(state._id, state._value);

    callback(t0, values);

    for (auto& state: states)
        if (state._scalar)
            state._value[0] = values.get_scalar(state._id) + h * values.get_scalar(state._deriv_id);
        else
            state._value = values.get_array(state._id) + h * values.get_array(state._deriv_id);
}

}
