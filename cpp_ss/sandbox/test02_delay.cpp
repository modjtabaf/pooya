
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
    SSModel(Submodel* parent) : Submodel(parent, "")
    {
        Node x("x");
        Node xd("xd");

        new Const(this, "TimeDelay", 2.7435, {"-time_delay"});
        new Const(this, "Initial", 0.0, {"-initial"});
        new Delay(this, "Delay", {x, "-time_delay", "-initial"}, xd);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model();
    auto x  = Node("x",  model);
    auto xd = Node("xd", model);
    auto ss_model = SSModel(&model);
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 10, 0.1);
        },
        [&](double t, const NodeIdValues& /*X*/, NodeIdValues& inputs) -> void
        {
            inputs.insert_or_assign(x, std::sin(M_PI * t / 5));
        });

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
