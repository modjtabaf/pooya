
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class SSModel : public Submodel
{
public:
    SSModel(Submodel* parent) : Submodel(parent, "pendulum")
    {
        auto   phi = parent->signal(  "phi");
        auto  dphi = parent->signal( "dphi");
        auto d2phi = parent->signal("d2phi");
        auto   tau = parent->signal(  "tau");
        auto     m = parent->signal(    "m");
        auto     g = parent->signal(    "g");
        auto     l = parent->signal(    "l");

        new Integrator(this, "dphi", d2phi, dphi, M_PI_4);
        new Integrator(this, "phi", dphi, phi);
        new Function(this, "sin(phi)",
            [](double /*t*/, const Value& x) -> Value
            {
                return x.sin();
            }, phi, 10);
        // new Sin(this, "sin(phi)", phi, 10);
        new MulDiv(this, "g\\l", "**/", {10, g, l}, 20);
        new MulDiv(this, "tau\\ml2", "*///", {tau, m, l, l}, 30);
        new AddSub(this, "d2phi", "+-", {30, 20}, d2phi);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model("test06");

    SignalValues parameters({
        {  "m", 0.2 },
        {  "l", 0.1 },
        {  "g", 9.81},
        {"tau", 0.13},
        }, model);

    auto phi  = model.signal("phi");
    auto dphi = model.signal("dphi");

    auto ss_model = SSModel(&model);
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        nullptr, parameters, rk4);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-8:8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi'\n";

    return 0;
}
