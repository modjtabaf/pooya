
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model("test01");
    auto x  = model.signal("x");
    auto xd = model.signal("xd");
    new Memory(&model, "memory", x, xd);

    History history(model);

    Simulator sim(model,
        [&](double t, Values& values) -> void
        {
            values.set(x, std::sin(M_PI * t / 5));
        });

    uint k = 0;
    double t;
    while (arange(k, t, 0, 10, 0.1))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    Gnuplot gp;
	gp << "set xrange [0:100]\n";
    gp << "set yrange [-1:1]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x',"
		<< gp.file1d(history[xd]) << "with lines title 'xd'\n";

    return 0;
}
