
#ifndef __HELPER_HPP__
#define __HELPER_HPP__

#include <vector>
#include <fstream>

#include "blocks.hpp"
#include "solver.hpp"

namespace blocks
{

using InputCallback = std::function<void(double, const NodeIdValues&, NodeIdValues&)>;
using TimeCallback  = std::function<bool(uint k, double& t)>;
using History       = std::map<Node::Id, MatrixXd>;

History run(Model& model, TimeCallback time_cb, InputCallback inputs_cb=nullptr, const NodeIdValues& parameters=NodeIdValues(), Solver stepper=nullptr);
bool arange(uint k, double& t, double t_init, double t_end, double dt);
void export_csv(const History& history, std::string filename);

}

#endif // __HELPER_HPP__
