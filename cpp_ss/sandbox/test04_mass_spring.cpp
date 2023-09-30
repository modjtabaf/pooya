
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
    SSModel(Submodel* parent) : Submodel(parent, "SSModel")
    {
        auto   x = parent->create_signal(  "x");
        auto  xd = parent->create_signal( "xd");
        auto xdd = parent->create_signal("xdd");

        new Integrator(this, "xd", xdd, xd, 0.1);
        new Integrator(this, "x", xd, x);
        new Gain(this, "-k\\m", -1.0/1.0, x, xdd);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model("test04");
    auto x  = model.create_signal("x");
    auto xd = model.create_signal("xd");
    auto ss_model = SSModel(&model);

    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        nullptr, SignalValues(), rk4);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    // export_csv(history, "mass_spring.csv");

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-0.15:0.15]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x',"
		<< gp.file1d(history[xd]) << "with lines title 'xd'\n";

    return 0;
}
