
#ifndef __HELPER_HPP__
#define __HELPER_HPP__

#include <vector>

#include "blocks.hpp"
#include "solver.hpp"

namespace blocks
{

using InputCallback = std::function<void(double, const VectorXd&, NamedValues&)>;
using TimeCallback  = std::function<bool(uint k, double& t)>;
using History       = std::map<std::string, MatrixXd>;

History run(Base& model, TimeCallback time_cb, InputCallback inputs_cb=nullptr, const NamedValues& parameters=NamedValues(), Solver stepper=nullptr);
bool arange(uint k, double& t, double t_init, double t_end, double dt);

}

#endif // __HELPER_HPP__
