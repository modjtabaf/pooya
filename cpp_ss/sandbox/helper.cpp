
#include <iostream>
#include <fstream>

#include "blocks.hpp"
#include "helper.hpp"

namespace blocks
{

History run(Model& model, TimeCallback time_cb, InputCallback inputs_cb, const NodeValues& parameters, Solver stepper)
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
                std::cout << "- " << c->name() << "\n";
                for (const auto& p: c->iports())
                    std::cout << "  - i: " << (values.get(p) ? " " : "*") <<  p << "\n";
                for (const auto& p: c->oports())
                    std::cout << "  - o: " << (values.get(p) ? " " : "*") <<  p << "\n";
            }
        }
        return n_processed;
    };

    History history;

    StatesInfo states;
    model.get_states(states);

    auto stepper_callback = [&](double t, Values& values) -> void
    {
        for (const auto& nv: parameters)
            values.set(nv.first, nv.second);
    
        if (inputs_cb)
            inputs_cb(t, values);

        process(t, values);
    };

    Values values(model.num_nodes());

    auto update_history = [&](double t, Values& values) -> void
    {
        model.step(t, values);

        if (history.empty())
        {
            history.reserve(values.size());
            for (Node::Id id = 0; id < model.num_nodes(); id++)
            {
                history[id] = Value();
                // if ((parameters.find(v.first) ==  parameters.end()) && (model.get_node_by_id(v.first)[0] != '-')) // todo: exclude parameters
                //     history.insert_or_assign(v.first, Value());
            }
        }

        auto& h = history[0]; // t
        auto nrows = h.rows() + 1;
        h.conservativeResize(nrows, NoChange);
        h(nrows - 1, 0) = t;
        for (Node::Id id = 0; id < model.num_nodes(); id++)
        {
            // if ((parameters.find(v.first) ==  parameters.end()) && (model.get_node_by_id(v.first)[0] != '-')) // todo: exclude parameters
            auto& h = history[id];
            h.conservativeResize(nrows, NoChange);
            const auto* v = values.get(id);
            assert(v);
            history[id].bottomRows<1>() = *v;
        }
    };

    if (states.size() > 0)
    {
        assert(stepper);
        uint k = -1;
        double t{0}, t1;

        while (time_cb(++k, t1))
        {
            if (k%100 == 0)
                std::cout << k << ": " << t1 << "\n";

            if (k == 0) // todo: why?
            {
                t = t1;
                continue;
            }

            stepper(stepper_callback, t, t1, states, model.num_nodes());

            values.invalidate();
            for (auto& state: states)
                values.set(state.first, state.second.first);
            for (const auto& nv: parameters)
                values.set(nv.first, nv.second);
            if (inputs_cb)
                inputs_cb(t, values);
            process(t, values);
            update_history(t, values);

            t = t1;
        }

        values.invalidate();
        for (auto& state: states)
            values.set(state.first, state.second.first);
        for (const auto& nv: parameters)
            values.set(nv.first, nv.second);
        if (inputs_cb)
            inputs_cb(t, values);
        process(t, values);
        update_history(t, values);
    }
    else
    {
        uint k = 0;
        double t;
        while (time_cb(k++, t))
        {
            values.invalidate();

            stepper_callback(t, values);

            update_history(t, values);
        }
    }

    return history;
}

bool arange(uint k, double& t, double t_init, double t_end, double dt)
{
    t = t_init + k * dt;
    return t <= t_end;
}

void export_csv(const History& history, std::string filename)
{
    if (history.size() == 0)
        return;

    std::ofstream ofs(filename);

    // header
    for (const auto& h: history)
        ofs << h.first << ",";
    ofs << "\n";

    // values
    auto n = history.begin()->second.size();
    for (int k = 0; k < n; k++)
    {
        for (const auto& h: history)
            ofs << h.second(k) << ",";
        ofs << "\n";
    }
}

}
