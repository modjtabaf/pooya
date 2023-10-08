/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __HELPER_HPP__
#define __HELPER_HPP__

#include <vector>
#include <fstream>

#include "pooya.hpp"
#include "solver.hpp"

namespace pooya
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

    void run(double t, double min_time_step = 1e-3, double max_time_step = 1);

    const Values& values() const {return _values;}
};

bool arange(uint k, double& t, double t_init, double t_end, double dt);

}

#endif // __HELPER_HPP__