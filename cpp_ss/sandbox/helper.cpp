
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

void run(Model& model, TimeCallback time_cb, InputCallback inputs_cb, OutputCallback outputs_cb, Solver stepper)
{
    auto process = [&](double t, Values& values) -> uint
    {
        uint n_processed = model._process(t, values, true);
        uint n;
        do
        {
            n = model._process(t, values, false);
            n_processed += n;
        } while(n);

        std::vector<const Base*> unprocessed;

        auto find_unprocessed_cb = [&] (const Base& c, uint32_t /*level*/) -> bool
        {
            if (!c.processed())
                unprocessed.push_back(&c);
            return true;
        };

        model.traverse(find_unprocessed_cb, 0);
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
    };

    StatesInfo states;
    model.get_states(states);

    auto stepper_callback = [&](double t, Values& values) -> void
    {
        if (inputs_cb)
            inputs_cb(t, values);

        process(t, values);
    };

    Values values(model.num_signals());

    if (states.size() > 0)
    {
        assert(stepper);
        uint k = -1;
        double t{0}, t1;

        auto call_outputs_cb = [&]() -> void
        {
            if (outputs_cb)
            {
                values.invalidate();
                for (auto& state: states)
                    values.set(state.first, state.second.first);
                if (inputs_cb)
                    inputs_cb(t, values);
                process(t, values);
                outputs_cb(k, t, values);
            }
        };

        while (time_cb(++k, t1))
        {
            if (k%100 == 0)
                std::cout << k << ": " << t1 << "\n";

            if (k > 0)
            {
                stepper(stepper_callback, t, t1, states, model.num_signals());
                model.step(t, values);
            }

            call_outputs_cb(); // todo: used only once. Move the code here

            t = t1;
        }
    }
    else
    {
        uint k = -1;
        double t;
        while (time_cb(++k, t))
        {
            values.invalidate();

            stepper_callback(t, values);
            model.step(t, values);

            if (outputs_cb)
                outputs_cb(k, t, values);
        }
    }
}

bool arange(uint k, double& t, double t_init, double t_end, double dt)
{
    t = t_init + k * dt;
    return t <= t_end;
}

}
