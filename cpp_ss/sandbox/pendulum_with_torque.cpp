
#include <iostream>
#include <math.h>
#include <vector>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class SSModel : public Submodel
{
public:
    SSModel() : Submodel("pendulum")
    {
        auto phi   = N("phi");
        auto dphi  = N("dphi");
        auto d2phi = N("d2phi");
        auto tau   = N("tau");
        auto m     = N("m");
        auto g     = N("g");
        auto l     = N("l");

        enter();
        {
            new Integrator("dphi", d2phi, dphi, M_PI_4);
            new Integrator("phi", dphi, phi);
            new Function("sin(phi)",
                [](double t, const Signal& x) -> Signal
                {
                    return x.sin();
                }, phi);
            // new Sin("sin(phi)", {phi});
            new MulDiv("g/l", "**/", {N(), g, l}, N("-1"));
            new MulDiv("", "*///", {tau, m, l, l}, {N("-2")});
            new AddSub("", "+-", {N("-2"), N("-1")}, {N("d2phi")});
        }
        exit();
    }
};

int main()
{
    NamedSignals parameters = {
        {"m", S(0.2)},
        {"l", S(0.1)},
        {"g", S(9.81)},
        {"tau", S(0.13)},
        };

    auto model = SSModel();
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        nullptr, parameters, rk4);

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-8:8]\n";
	gp << "plot" << gp.file1d(history["phi"]) << "with lines title 'phi',"
	    << gp.file1d(history["dphi"]) << "with lines title 'dphi'\n";

    return 0;
}
