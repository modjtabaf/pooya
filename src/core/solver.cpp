
#include <iostream>
#include <ostream>
#include <type_traits>
#include "solver.hpp"

namespace pooya
{

#define CALC_K(K, t, v) \
    X.set_states(v); \
    callback(t, X); \
    static Array K; \
    K = h * X.derivs();

const Array& rk4(const Model& model, SolverCallback callback, double t0, const Array& v0, double t1, double& new_h)
{
    static Values X(model);
    static Array v1(v0.size());

    double h = new_h = t1 - t0;

    // K1 = h * f(t0, v0)
    CALC_K(K1, t0, v0)

    // K2 = h * f(t0 + h/2, v0 + K1/2)
    CALC_K(K2, t0 + h/2, v0 + K1 / 2)

    // K3 = h * f(t0 + h/2, v0 + K2/2)
    CALC_K(K3, t0 + h/2, v0 + K2 / 2)

    // K4 = h * f(t0 + h, v0 + K3)
    CALC_K(K4, t0 + h, v0 + K3)

    // v1 = v0 + K1/6 + K2/3 + K3/3 + K4/6
    v1 = v0 + K1 / 6 + K2 / 3 + K3 / 3 + K4 / 6;

    return v1;
}

// source: https://ece.uwaterloo.ca/~dwharder/NumericalAnalysis/14IVPs/rkf45/complete.html
const Array& rkf45(const Model& model, SolverCallback callback, double t0, const Array& v0, double t1, double& new_h)
{
    static Values X(model);
    static Array YT(v0.size());
    static Array ZT(v0.size());

    double h = t1 - t0;
    double eps_abs = 0.001;

    // K1 = h * f(t0, v0)
    CALC_K(K1, t0, v0)

    // K2 = h * f(t0 + h/4, v0 + K1/4)
    CALC_K(K2, t0 + h/4, v0 + 1./4 * K1)

    // K3 = h * f(t0 + 3/8 * h, v0 + 3/8 * ( 1/4 * K1 + 3/4 * K2))
    CALC_K(K3, t0 + 3./8 * h, v0 + (3./8 * 1./4) * K1 + (3./8 * 3./4) * K2)

    // K4 = h * f(t0 + 12/13 * h, v0 + 12/13 * (161/169 * K1 - 600/169 * K2 + 608/169 * K3))
    CALC_K(K4, t0 + 12./13 * h, v0 + (12./13 * 161./169) * K1 - (12./13 * 600./169) * K2 + (12./13 * 608./169) * K3)

    // K5 = h * f(t0 + h, v0 + 8341/4104 * K1 - 32832/4104 * K2 + 29440/4104 * K3 - 845/4104 * K4)
    CALC_K(K5, t1, v0 + (8341./4104) * K1 - (32832./4104) * K2 + (29440./4104) * K3 - (845./4104) * K4)

    // K6 = h * f(t0 + 1/2 * h, v0 + 1/2 * (-6080/10260 * K1 + 41040/10260 * K2 - 28352/10260 * K3 + 9295/10260 * K4 - 5643/10260 * K5))
    CALC_K(K6, t0 + 1./2 * h, v0 - (1./2 * 6080./10260) * K1 + (1./2 * 41040./10260) * K2 - (1./2 * 28352./10260) * K3 + (1./2 * 9295./10260) * K4 - (1./2 * 5643./10260) * K5)

    // yt := y[i - 1] + h*(  2375*K[1] +  11264*K[3] +  10985*K[4] - 4104*K[5] )/20520;
    YT = v0 + (2375./20520) * K1 + (11264./20520) * K3 + (10985./20520) * K4 - (4104./20520) * K5;

    // zt := y[i - 1] + h*( 33440*K[1] + 146432*K[3] + 142805*K[4] - 50787*K[5] + 10260*K[6] )/282150;
    ZT = v0 + (33440./282150) * K1 + (146432./282150) * K3 + (142805./282150) * K4 - (50787./282150) * K5 + (10260./282150) * K6;

    double max_abs = (YT - ZT).abs().max(0)(0, 0);
    double s4 = max_abs > 0 ? (eps_abs * h) / (2 * max_abs) : 1;

    if (s4 < 0.31640625) // 0.31640625 == 0.75 ^ 4
        new_h = h / 2;
    else if (s4 > 5.0625) // 5.0625 == 1.5 ^ 4
        new_h = 2 * h;
    else
        new_h = h;

    return YT;
}

const Array& euler(const Model& model, SolverCallback callback, double t0, const Array& v0, double t1, double& new_h)
{
    static Values X(model);
    static Array v1(v0.size());

    double h = new_h = t1 - t0;

    // K = h * f(t0, v0)
    CALC_K(K, t0, v0)

    // v1 = v0 + K
    v1 = v0 + K;

    return v1;
}

}
