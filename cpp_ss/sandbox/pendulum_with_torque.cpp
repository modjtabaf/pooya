
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
        Node   phi(  "phi");
        Node  dphi( "dphi");
        Node d2phi("d2phi");
        Node tau("tau");
        Node m("m");
        Node g("g");
        Node l("l");

        enter();
        {
            new Integrator("dphi", d2phi, dphi, M_PI_4);
            new Integrator("phi", dphi, phi);
            new Function("sin(phi)",
                [](double t, const Signal& x) -> Signal
                {
                    return x.sin();
                }, phi);
            // new Sin("sin(phi)", phi);
            new MulDiv("g/l", "**/", {Node(), g, l}, -1);
            new MulDiv("", "*///", {tau, m, l, l}, -2);
            new AddSub("", "+-", {Node(-2), Node(-1)}, "d2phi");
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
