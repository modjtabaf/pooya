
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
        auto g     = N("g");
        auto l     = N("l");

        enter();
        {
            new Integrator("dphi", d2phi, dphi);
            new Integrator("phi", dphi, phi, M_PI_4);
            new Function("sin(phi)",
                [](double t, const Signal& x) -> Signal
                {
                    return x.sin();
                }, phi);
            // new Sin("sin(phi)", {phi}, {N()});
            new MulDiv("-g/l", "**/", {N(), g, l}, d2phi, -1);
        }
        exit();
    }
};

int main()
{
    NamedSignals parameters = {
        {"l", S(0.1)},
        {"g", S(9.81)},
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
    gp << "set yrange [-0.8:0.8]\n";
	gp << "plot" << gp.file1d(history["phi"]) << "with lines title 'x'\n";

    return 0;
}
