
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
    SSModel(Submodel* parent, const Signal& x, const Signal& xd) : Submodel(parent, "SSModel", {x, xd})
    {
        auto xdd = signal("xdd");

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
    auto x  = model.signal("x");
    auto xd = model.signal("xd");
    new SSModel(&model, x, xd);

    History history(model);

    run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
        },
        nullptr,
        [&](uint k, double t, Values& values) -> void
        {
            history.update(k, t, values);
        },
        rk4);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    history.export_csv("mass_spring.csv");

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-0.15:0.15]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x',"
		<< gp.file1d(history[xd]) << "with lines title 'xd'\n";

    return 0;
}
