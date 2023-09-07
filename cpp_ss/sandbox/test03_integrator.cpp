
#include <iostream>
#include <cmath>
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

    auto model = Model();
    auto x  = Node("x",  model);
    auto xd = Node("xd", model);
    auto blk = Integrator(&model, "", xd, x, 1.0);
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 10, 0.1);
        },
        [&](double t, const NodeIdValues& /*X*/, NodeIdValues& inputs) -> void
        {
            inputs.insert_or_assign(xd, t < 3 or t > 7 ? 1 : 0);
        }, NodeIdValues(), rk4);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    Gnuplot gp;
	gp << "set xrange [0:100]\n";
	gp << "set yrange [-1:7]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x',"
		<< gp.file1d(history[xd]) << "with lines title 'xd'\n";

    return 0;
}
