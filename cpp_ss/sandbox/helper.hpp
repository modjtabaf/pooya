
#ifndef __HELPER_HPP__
#define __HELPER_HPP__

#include <vector>
#include <fstream>

#include "blocks.hpp"
#include "solver.hpp"

namespace blocks
{

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

using InputCallback  = std::function<void(double, Values&)>;

class Simulator
{
protected:
    Model& _model;
    double _t_prev{0};
    InputCallback _inputs_cb;
    Values _values;
    StatesInfo _states;
    Solver _stepper;
    bool _first_iter{true};

    uint _process(double t, Values& values);
    void _update_values(double t);

public:
    Simulator(Model& model, InputCallback inputs_cb = nullptr, Solver stepper = nullptr);

    void run(double t);

    const Values& values() const {return _values;}
};

bool arange(uint k, double& t, double t_init, double t_end, double dt);

}

#endif // __HELPER_HPP__
