
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
    SSModel() : Submodel(nullptr, "pendulum")
    {
        Node   phi(  "phi");
        Node  dphi( "dphi");
        Node d2phi("d2phi");
        Node tau("tau");
        Node m("m");
        Node g("g");
        Node l("l");

        new Integrator(this, "dphi", d2phi, dphi, M_PI_4);
        new Integrator(this, "phi", dphi, phi);
        new Function(this, "sin(phi)",
            [](double /*t*/, const Value& x) -> Value
            {
                return x.sin();
            }, phi);
        // new Sin(this, "sin(phi)", phi);
        new MulDiv(this, "g/l", "**/", {"", g, l}, -1);
        new MulDiv(this, "", "*///", {tau, m, l, l}, -2);
        new AddSub(this, "", "+-", {-2, -1}, "d2phi");
    }
};

int main()
{
    NodeValues parameters = {
        {  "m", 0.2 },
        {  "l", 0.1 },
        {  "g", 9.81},
        {"tau", 0.13},
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
