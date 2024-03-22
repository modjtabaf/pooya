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
#include <string>
#include "pooya.hpp"
#include "util.hpp"
#include "helper.hpp"

namespace pooya
{

void History::update(uint k, double t, const Values& values)
{
    pooya_trace("k = " + std::to_string(k));
    if (empty())
    {
        for (const auto& vi: values.value_infos())
        {
            if (vi.is_scalar())
                insert_or_assign(vi._si, Array(_nrows_grow));
            else
                insert_or_assign(vi._si, Eigen::MatrixXd(_nrows_grow, vi.size()));
        }
    }

    if (k >= _time.rows())
        _time.conservativeResize(k + _nrows_grow, Eigen::NoChange);
    _time(k, 0) = t;
    for (const auto& vi: values.value_infos())
    {
        auto& h = at(vi._si);
        if (k >= h.rows())
            h.conservativeResize(k + _nrows_grow, Eigen::NoChange);
        bool valid = values.valid(vi._si);
        if (vi.is_int())
            h(k, 0) = valid ? values.get<int>(vi._si) : 0;
        if (vi.is_bool())
            h(k, 0) = valid ? values.get<bool>(vi._si) : 0;
        else if (vi.is_scalar())
            h(k, 0) = valid ? values.get<double>(vi._si) : 0;
        else
        {
            if (valid)
                h.row(k) = values.get<Array>(vi._si);
            else
                h.row(k).setZero();
        }
    }

    if ((_bottom_row == uint(-1)) || (k > _bottom_row))
        _bottom_row = k;
}

void History::shrink_to_fit()
{
    pooya_trace0;
    const uint nrows = _bottom_row + 1;

    if (nrows >= _time.rows()) // practically, nrows can't be the greater
        return;

    _time.conservativeResize(nrows, Eigen::NoChange);
    for (auto& p: *this)
        p.second.conservativeResize(nrows, Eigen::NoChange);
}

void History::export_csv(std::string filename)
{
    pooya_trace("filename = " + filename);
    if (size() == 0)
        return;

    std::ofstream ofs(filename);

    // header
    ofs << "time";
    for (const auto& h: *this)
    {
        if (h.first)
        {
            if (h.first->as_scalar())
                ofs << "," << h.first->_full_name;
            else
            {
                auto sig = h.first->as_array();
                for (std::size_t k=0; k < sig->_size; k++)
                    ofs << "," << h.first->_full_name << "[" << k << "]";
            }
        }
    }
    ofs << "\n";

    // values
    auto n = time().size();
    for (int k = 0; k < n; k++)
    {
        ofs << time()(k);
        for (const auto& h: *this)
        if (h.first)
        {
            if (h.first->as_scalar())
                ofs << "," << h.second(k);
            else
            {
                auto sig = h.first->as_array();
                for (std::size_t j=0; j < sig->_size; j++)
                    ofs << "," << h.second(k, j);
            }
        }
        ofs << "\n";
    }
}

bool arange(uint k, double& t, double t_init, double t_end, double dt)
{
    t = t_init + k * dt;
    return t <= t_end;
}

Simulator::Simulator(Model& model, InputCallback inputs_cb, StepperBase* stepper, bool reuse_order) :
    _model(model), _inputs_cb(inputs_cb), _values(model), _stepper(stepper), _reuse_order(reuse_order)
{
    pooya_trace("model: " + model.full_name());
}

uint Simulator::_process(double t, Values& values)
{
    pooya_trace("t: " + std::to_string(t));
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

#if !defined(POOYA_NDEBUG)
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
        std::cout << "\n-- unprocessed blocks detected:\n";
        for (const auto& c: unprocessed)
        {
            std::cout << "- " << c->full_name() << "\n";
            for (auto sig: c->dependencies())
            {
                if (values.valid(sig))
                    continue;
                std::cout << "  - " << sig->_full_name << "\n";
            }
        }
    }
#endif // !defined(POOYA_NDEBUG)

    return n_processed;
}

void Simulator::init(double t0)
{
    pooya_trace("t0: " + std::to_string(t0));

    _t_prev = t0;

    if (_values.num_state_variables() > 0)
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

    _values.invalidate();
    _inputs_cb ? _inputs_cb(_model, t0, _values) : _model.input_cb(t0, _values);
    _model.pre_step(t0, _values);
    _process(t0, _values);
    _model.post_step(t0, _values);

    _initialized = true;
}

void Simulator::run(double t, double min_time_step, double max_time_step)
{
    pooya_trace("t: " + std::to_string(t));
    auto stepper_callback = [&](double t, Values& values) -> void
    {
        pooya_trace("t: " + std::to_string(t));
        _inputs_cb ? _inputs_cb(_model, t, values) : _model.input_cb(t, values);
        _process(t, values);
    };

    if (!_initialized)
    {
        // treat the first call as the initialization call if init(t0) was not called explicitely
        init(t);
        return;
    }

    if (_values.num_state_variables() > 0)
    {
        if (t <_t_prev)
            util::pooya_throw_exception(__FILE__, __LINE__,
                "Simulation cannot go back in time. Aborting!\n  "
                "- current time  = " + std::to_string(t) + "\n  "
                "- previous time = " + std::to_string(_t_prev) + "\n");

        if (t == _t_prev)
        {
            util::pooya_show_warning(__FILE__, __LINE__, 
                "Repeated simulation step:\n  "
                "- time = " + std::to_string(t) + "\n");

            _values.invalidate();
            _inputs_cb ? _inputs_cb(_model, t, _values) : _model.input_cb(t, _values);
            _model.pre_step(t, _values);
            _state_variables = _values.state_variables();
        }
        else
        {
            assert(_stepper);

            assert(min_time_step > 0);
            assert(max_time_step > min_time_step);

            double new_h;
            double t1 = _t_prev;
            double t2 = t;
            bool force_accept = false;
            while (t1 < t)
            {
                _values.invalidate();
                _inputs_cb ? _inputs_cb(_model, t1, _values) : _model.input_cb(t1, _values);
                _model.pre_step(t1, _values);
                _state_variables_orig = _values.state_variables();

                _stepper->step(stepper_callback, t1, _state_variables_orig, t2, _state_variables, new_h);

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
                        _values.reset_with_state_variables(_state_variables);
                        _inputs_cb ? _inputs_cb(_model, t, _values) : _model.input_cb(t, _values);
                        _process(t1, _values);
                        _model.post_step(t1, _values);
                    }
                }
                else
                {
                    // redo this step
                    force_accept = new_h <= min_time_step;
                    new_h = std::max(min_time_step, std::min(new_h, max_time_step));
                    t2 = t1 + new_h;
                }
            }
        }
    }

    _values.reset_with_state_variables(_state_variables);
    _inputs_cb ? _inputs_cb(_model, t, _values) : _model.input_cb(t, _values);
    _process(t, _values);
    _model.post_step(t, _values);

    _t_prev = t;
}

}
