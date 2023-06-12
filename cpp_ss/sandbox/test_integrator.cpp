
#include <iostream>
#include <cmath>
#include <vector>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

int main()
{
    auto blk = Integrator("", N("xd"), N("x"), 1.0);
    auto history = run(blk,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 10, 0.1);
        },
        [](double t, const VectorXd& x, NamedValues& inputs) -> void
        {
            auto foo = VectorXd(1);
            foo << (t < 3 or t > 7 ? 1 : 0);
            inputs.insert_or_assign("xd", foo);
        }, NamedValues(), rk4);

    Gnuplot gp;
	gp << "set xrange [0:100]\n";
	gp << "set yrange [-1:7]\n";
	gp << "plot" << gp.file1d(history["x"]) << "with lines title 'x',"
		<< gp.file1d(history["xd"]) << "with lines title 'xd'\n";

    return 0;
}
