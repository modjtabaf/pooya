
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

        auto g = parameter("g");
        auto l = parameter("l");

        new Integrator(this, "dphi", d2phi, dphi);
        new Integrator(this, "phi", dphi, phi, M_PI_4);
        // new Function(this, "sin(phi)",
        //     [](double /*t*/, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi, 1);
        new Sin(this, "sin(phi)", phi, 10);
        new MulDiv(this, "-g\\l", "**/", {10, g, l}, d2phi, -1);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model("test05");

    SignalValues parameters({
        {"l", 0.1 },
        {"g", 9.81},
        }, model);

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

    auto phi = ss_model.signal("phi");

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-0.8:0.8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'x'\n";

    return 0;
}
