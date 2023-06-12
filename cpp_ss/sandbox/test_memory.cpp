
#include <iostream>
#include <math.h>
#include <vector>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

int main()
{
    auto memory = Memory("", N("x"), N("xd"));
    auto history = run(memory,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 10, 0.1);
        },
        [](double t, const VectorXd& x, NamedValues& inputs) -> void
        {
            auto foo = VectorXd(1);
            foo << std::sin(M_PI * t / 5);
            inputs.insert_or_assign("x", foo);
        });

    Gnuplot gp;
	gp << "set xrange [0:100]\n";
    gp << "set yrange [-1:1]\n";
	gp << "plot" << gp.file1d(history["x"]) << "with lines title 'x',"
		<< gp.file1d(history["xd"]) << "with lines title 'xd'\n";

    return 0;
}
