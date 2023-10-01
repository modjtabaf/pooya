
#ifndef __HELPER_HPP__
#define __HELPER_HPP__

#include <vector>
#include <fstream>

#include "blocks.hpp"
#include "solver.hpp"

namespace blocks
{

using TimeCallback   = std::function<bool(uint k, double& t)>;
using InputCallback  = std::function<void(double, Values&)>;
using OutputCallback = std::function<void(uint k, double, Values&)>;

class History : public std::unordered_map<Signal::Id, MatrixXd>
{
public:
    static constexpr Signal::Id time_id = Signal::NoId;

protected:
    const Model& _model;

public:
    History(const Model& model) : _model(model) {}

    void update(uint k, double t, const Values& values);
    void export_csv(std::string filename);
};

void run(Model& model, TimeCallback time_cb, InputCallback inputs_cb=nullptr, OutputCallback outputs_cb=nullptr, Solver stepper=nullptr);
bool arange(uint k, double& t, double t_init, double t_end, double dt);

}

#endif // __HELPER_HPP__
