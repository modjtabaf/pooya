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

#include <iostream>
#include <fstream>

#include "blocks.hpp"
#include "helper.hpp"

namespace blocks
{

void History::update(uint k, double t, const Values& values)
{
    if (empty()) // todo: consider reserving space for values
    {
        reserve(values.size() + 1);
        insert_or_assign(time_id, Value()); // t
        for (Signal::Id id = 0; id < _model.num_signals(); id++)
            insert_or_assign(id, Value());
    }

    auto& h = at(time_id); // t
    if (k >= h.rows())
        h.conservativeResize(k + 1, NoChange);
    h(k, 0) = t;
    for (Signal::Id id = 0; id < _model.num_signals(); id++)
    {
        auto& h = at(id);
        if (k >= h.rows())
            h.conservativeResize(k + 1, NoChange);
        const auto* v = values.get(id);
        assert(v);
        h.row(k) = *v;
    }
}

void History::export_csv(std::string filename)
{
    if (size() == 0)
        return;

    std::ofstream ofs(filename);

    // header
    ofs << "time";
    for (const auto& h: *this)
        if (h.first != time_id)
            ofs << "," << _model.get_signal_by_id(h.first);
    ofs << "\n";

    // values
    auto n = at(time_id).size();
    for (int k = 0; k < n; k++)
    {
        ofs << at(time_id)(k);
        for (const auto& h: *this)
            if (h.first != time_id)
                ofs << "," << h.second(k);
        ofs << "\n";
    }
}

bool arange(uint k, double& t, double t_init, double t_end, double dt)
{
    t = t_init + k * dt;
    return t <= t_end;
}

Simulator::Simulator(Model& model, InputCallback inputs_cb, Solver stepper) :
    _model(model), _inputs_cb(inputs_cb), _values(model.num_signals()), _stepper(stepper)
{
    model.get_states(_states);
}

uint Simulator::_process(double t, Values& values)
{
    uint n_processed = _model._process(t, values, true);
    uint n;
    do
    {
        n = _model._process(t, values, false);
        n_processed += n;
    } while(n);

    std::vector<const Base*> unprocessed;

    auto find_unprocessed_cb = [&] (const Base& c, uint32_t /*level*/) -> bool
    {
        if (!c.processed())
            unprocessed.push_back(&c);
        return true;
    };

    _model.traverse(find_unprocessed_cb, 0);
    if (unprocessed.size())
    {
        std::cout << "-- unprocessed blocks detected:\n";
        for (const auto& c: unprocessed)
        {
            std::cout << "- " << c->full_name() << "\n";
            for (const auto& p: c->iports())
                std::cout << "  - i: " << (values.get(p) ? " " : "*") <<  p.full_name() << "\n";
            for (const auto& p: c->oports())
                std::cout << "  - o: " << (values.get(p) ? " " : "*") <<  p.full_name() << "\n";
        }
    }
    return n_processed;
}

void Simulator::run(double t, double min_time_step, double max_time_step)
{
    auto stepper_callback = [&](double t, Values& values) -> void
    {
        if (_inputs_cb)
            _inputs_cb(t, values);
        _process(t, values);
    };

    if (_states.size() > 0)
    {
        assert(_stepper);
        assert(t >= _t_prev);
        assert(min_time_step > 0);
        assert(max_time_step > min_time_step);

        StatesInfo states_orig(_states);

        if (!_first_iter)
        {
            double new_h;
            double t1 = _t_prev;
            double t2 = t;

            StatesInfo states_orig(_states);
            bool force_accept = false;
            while (t1 < t)
            {
                _stepper(stepper_callback, t1, t2, _states, _model.num_signals(), new_h);

                double h = t2 - t1;
                if (force_accept || (new_h >= h) || (h <= min_time_step))
                {
                    // accept this step
                    force_accept = false;
                    new_h = std::max(min_time_step, std::min(new_h, max_time_step));
                    t1 = t2;
                    t2 = std::min(t1 + new_h, t);

                    if (t1 < t)
                    {
                        _values.invalidate();
                        for (auto& state: _states)
                            _values.set(state.first, state.second.first);
                        if (_inputs_cb)
                            _inputs_cb(t2, _values);
                        _process(t2, _values);
                        _model.step(t2, _values);
                    }
                }
                else
                {
                    // redo this step
                    force_accept = new_h <= min_time_step;
                    new_h = std::max(min_time_step, std::min(new_h, max_time_step));
                    auto it = states_orig.begin();
                    for (auto& state: _states)
                        state.second.first = (it++)->second.first;
                    t2 = t1 + new_h;
                }
            }
            
        }
    }

    if (_first_iter)
        _first_iter = false;

    _values.invalidate();
    for (auto& state: _states)
        _values.set(state.first, state.second.first);
    if (_inputs_cb)
        _inputs_cb(t, _values);
    _process(t, _values);
    _model.step(t, _values);

    _t_prev = t;
}

}
