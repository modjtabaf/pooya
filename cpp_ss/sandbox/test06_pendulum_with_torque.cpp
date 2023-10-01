
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
        auto   phi = signal(  "phi");
        auto  dphi = signal( "dphi");
        auto d2phi = signal("d2phi");

        auto tau = parameter("tau");
        auto m = parameter("m");
        auto g = parameter("g");
        auto l = parameter("l");

        new Integrator(this, "dphi", d2phi, dphi, M_PI_4);
        new Integrator(this, "phi", dphi, phi);
        new Function(this, "sin(phi)",
            [](double /*t*/, const Value& x) -> Value
            {
                return x.sin();
            }, phi, signal(10));
        // new Sin(this, "sin(phi)", phi, signal(10));
        new MulDiv(this, "g\\l", "**/", {signal(10), g, l}, signal(20));
        new MulDiv(this, "tau\\ml2", "*///", {tau, m, l, l}, signal(30));
        new AddSub(this, "d2phi", "+-", {signal(30), signal(20)}, d2phi);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model("test06");
    new SSModel(&model);

    auto m = model.signal("m");
    auto l = model.signal("l");
    auto g = model.signal("g");
    auto tau = model.signal("tau");

    History history(model);

    run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        [&](double /*t*/, Values& values) -> void
        {
            values.set(m, 0.2);
            values.set(l, 0.1);
            values.set(g, 9.81);
            values.set(tau, 0.13);
        },
        [&](uint k, double t, Values& values) -> void
        {
            history.update(k, t, values);
        },
        rk4);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto  phi = model.find_signal("/test06/pendulum.phi");
    auto dphi = model.find_signal("/test06/pendulum.dphi");

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-8:8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi'\n";

    return 0;
}
