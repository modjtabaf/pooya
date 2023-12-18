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
#include "pooya.hpp"
#include "helper.hpp"

namespace pooya
{

void History::update(uint k, double t, const Values& values)
{
    auto n = _model.signal_registry().num_signals();
    if (empty())
    {
        insert_or_assign(time_id, Array(_nrows_grow)); // t
        for (Signal::Id id = 0; id < n; id++)
        {
            const auto& vi = values.get_value_info(id);
            if (vi._scalar)
                insert_or_assign(id, Array(_nrows_grow));
            else
                insert_or_assign(id, Eigen::MatrixXd(_nrows_grow, vi._array.size()));
        }
    }

    auto& h = at(time_id); // t
    if (k >= h.rows())
        h.conservativeResize(k + _nrows_grow, Eigen::NoChange);
    h(k, 0) = t;
    for (Signal::Id id = 0; id < n; id++)
    {
        const auto& vi = values.get_value_info(id);
        auto& h = at(id);
        if (k >= h.rows())
            h.conservativeResize(k + _nrows_grow, Eigen::NoChange);
        if (vi._scalar)
            h(k, 0) = values.get_scalar(id);
        else
            h.row(k) = values.get_array(id);
    }

    if ((_bottom_row == uint(-1)) || (k > _bottom_row))
        _bottom_row = k;
}

void History::shrink_to_fit()
{
    const uint nrows = _bottom_row + 1;

    auto& h = at(time_id); // t
    if (nrows >= h.rows()) // practically, nrows can't be the greater
        return;

    h.conservativeResize(nrows, Eigen::NoChange);
    auto n = _model.signal_registry().num_signals();
    for (Signal::Id id = 0; id < n; id++)
        at(id).conservativeResize(nrows, Eigen::NoChange);
}

void History::export_csv(std::string filename)
{
    if (size() == 0)
        return;

    std::ofstream ofs(filename);
    const auto& sig_reg = _model.signal_registry();

    // header
    ofs << "time";
    for (const auto& h: *this)
        if (h.first != time_id)
            ofs << "," << sig_reg.get_signal_by_id(h.first).first;
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

Simulator::Simulator(Model& model, InputCallback inputs_cb, Solver stepper, bool reuse_order) :
    _model(model), _inputs_cb(inputs_cb), _values(model), _stepper(stepper), _reuse_order(reuse_order)
{
    model.get_states(_states_info);
    _states_info.lock();
    new (&_values) Values(model, _states_info);
}

uint Simulator::_process(double t, Values& values)
{
    _model._mark_unprocessed();

    uint n_processed = 0;

    if (_reuse_order)
    {
        _new_po->clear();

        bool any_processed;
        do
        {
            any_processed = false;
            for (auto* base: *_current_po)
            {
                if (base->processed())
                    continue;
                n_processed += base->_process(t, values, false);
                if (base->processed())
                {
                    _new_po->push_back(base);
                    any_processed = true;
                }
            }
        } while(any_processed);

        for (auto* base: *_current_po)
        {
            if (!base->processed())
                _new_po->push_back(base);
        }

        assert(_current_po->size() == _new_po->size());
        assert(_current_po->size() == _current_po->capacity());
        assert(_new_po->size() == _new_po->capacity());

        std::swap(_current_po, _new_po);
    }
    else
    {
        uint n;
        do
        {
            n = _model._process(t, values);
            n_processed += n;
        } while(n);
    }

    std::vector<const Block*> unprocessed;

    auto find_unprocessed_cb = [&] (const Block& c, uint32_t /*level*/) -> bool
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
                std::cout << "  - i: " << (values.valid(p) ? " " : "*") <<  p.full_name() << "\n";
            for (const auto& p: c->oports())
                std::cout << "  - o: " << (values.valid(p) ? " " : "*") <<  p.full_name() << "\n";
        }
    }
    return n_processed;
}

void Simulator::run(double t, double min_time_step, double max_time_step)
{
    auto stepper_callback = [&](double t, Values& values) -> void
    {
        _inputs_cb ? _inputs_cb(_model, t, values) : _model.input_cb(t, values);
        _process(t, values);
    };

    if (_states_info.size() > 0)
    {
        assert(_stepper);
        assert(t >= _t_prev);
        assert(min_time_step > 0);
        assert(max_time_step > min_time_step);

        StatesInfo states_info_orig(_states_info);

        if (_first_iter)
        {
            if (_reuse_order)
            {
                assert(_processing_order1.empty());
                assert(_processing_order2.empty());

                uint num_blocks = 0;
                auto enum_blocks_cb = [&] (Block& c, uint32_t /*level*/) -> bool
                {
                    num_blocks++;
                    return true;
                };
                _model.traverse(enum_blocks_cb, 0);

                _processing_order1.reserve(num_blocks);
                _processing_order2.reserve(num_blocks);

                auto add_blocks_cb = [&] (Block& c, uint32_t /*level*/) -> bool
                {
                    _processing_order1.push_back(&c);
                    return true;
                };
                _model.traverse(add_blocks_cb, 0);

                _current_po = &_processing_order1;
                _new_po = &_processing_order2;
            }
        }
        else
        {
            double new_h;
            double t1 = _t_prev;
            double t2 = t;

            StatesInfo states_info_orig(_states_info);
            bool force_accept = false;
            while (t1 < t)
            {
                _stepper(_model, stepper_callback, t1, t2, _states_info, new_h);

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
                        _values.set_states(_states_info.value());
                        states_info_orig.set_value(_states_info.value());
                        _inputs_cb ? _inputs_cb(_model, t, _values) : _model.input_cb(t, _values);
                        _process(t1, _values);
                        _model.step(t1, _values);
                    }
                }
                else
                {
                    // redo this step
                    force_accept = new_h <= min_time_step;
                    new_h = std::max(min_time_step, std::min(new_h, max_time_step));
                    _states_info.set_value(states_info_orig.value());
                    t2 = t1 + new_h;
                }
            }
            
        }
    }

    if (_first_iter)
        _first_iter = false;

    _values.invalidate();
    _values.set_states(_states_info.value());
    _inputs_cb ? _inputs_cb(_model, t, _values) : _model.input_cb(t, _values);
    _process(t, _values);
    _model.step(t, _values);

    _t_prev = t;
}

}
