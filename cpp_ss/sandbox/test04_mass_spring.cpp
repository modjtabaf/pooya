
#include <iostream>
#include <math.h>
#include <vector>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class SSModel : public Submodel
{
public:
    SSModel() : Submodel(nullptr, "")
    {
        Node x("x");
        Node xd("xd");
        Node xdd("xdd");

        new Integrator(this, "xd", xdd, xd, 0.1);
        new Integrator(this, "x", xd, x);
        new Gain(this, "-k/m", -1.0/1.0, x, xdd);
    }
};

int main()
{
    auto model = SSModel();
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 5, 0.01);
            // return arange(k, t, 0, 0.02, 0.01);
            // return arange(k, t, 0, 0.01, 0.01);
        },
        nullptr, NodeValues(), rk4);

    export_csv(history, "mass_spring.csv");

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-0.15:0.15]\n";
	gp << "plot" << gp.file1d(history["x"]) << "with lines title 'x',"
		<< gp.file1d(history["xd"]) << "with lines title 'xd'\n";

    return 0;
}
