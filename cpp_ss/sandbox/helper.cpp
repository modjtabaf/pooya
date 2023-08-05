
#include <iostream>

#include "blocks.hpp"
#include "helper.hpp"

namespace blocks
{

History run(Base& model, TimeCallback time_cb, InputCallback inputs_cb, const NodeValues& parameters, Solver stepper)
{
    auto process = [](Base& model, double t, NodeValues& x) -> uint
    {
        uint n_processed = model._process(t, x, true);
        uint n;
        do
        {
            n = model._process(t, x, false);
            n_processed += n;
        } while(n);

        std::vector<const Base*> unprocessed;

        auto find_unprocessed_cb = [&] (const Base& c) -> bool
        {
            if (not c.is_processed())
                unprocessed.push_back(&c);
            return true;
        };

        model.traverse(find_unprocessed_cb);
        if (unprocessed.size())
        {
            std::cout << "-- unprocessed blocks detected:\n";
            for (const auto& c: unprocessed)
            {
                std::cout << "- " << c->name() << "\n";
                for (const auto& p: c->iports())
                    std::cout << "  - i: " << (x.find(p) == x.first.end() ? "*" : " ") <<  p << "\n";
                for (const auto& p: c->oports())
                    std::cout << "  - o: " << (x.find(p) == x.first.end() ? "*" : " ") <<  p << "\n";
            }
        }
        return n_processed;
    };

    History history;
    NodeValues inputs;

    States states;
    model.get_states(states);

    auto stepper_callback = [&](double t, const NodeValues& x) -> Values
    {
        NodeValues y(x);

        y.join(parameters);
        y.join(inputs);

        process(model, t, y);

        Values ret;
        ret.reserve(x.first.size());
        for (const auto& state: std::get<2>(states))
        {
            auto& deriv = y.at(state);
            ret.push_back(deriv);
        }

        return ret;
    };

    NodeValues x(std::get<0>(states), std::get<1>(states));

    auto update_history = [&](double t, const NodeValues& x, const NodeValues& inputs) -> void
    {
        NodeValues y(x);

        y.join(parameters);
        y.join(inputs);

        process(model, t, y);

        model.step(t, y);

        if (history.empty())
        {
            history.insert_or_assign("t", Value());
            for (const auto& v: y.first)
                if ((parameters.find(v) ==  parameters.first.end()) && (v[0] != '-'))
                    history.insert_or_assign(v, Value());
        }

        auto& h = history["t"];
        auto nrows = h.rows() + 1;
        h.conservativeResize(nrows, NoChange);
        h(nrows - 1, 0) = t;
        uint k = 0;
        for (const auto& v: y.first)
        {
            if ((parameters.find(v) ==  parameters.first.end()) && (v[0] != '-'))
            {
                auto& h = history[v];
                h.conservativeResize(nrows, NoChange);
                history[v].bottomRows<1>() = y.at(v);
            }
            k++;
        }
    };

    if (std::get<0>(states).size())
    {
        assert(stepper);
        uint k = 0;
        double t, t1;
        while (time_cb(k, t1))
        {
            if (k%100 == 0)
                std::cout << k << ": " << t1 << "\n";

            if (k == 0)
            {
                t = t1;
                k = 1;
                continue;
            }

            if (inputs_cb)
                inputs_cb(t, x, inputs);
            update_history(t, x, inputs);
            x = stepper(stepper_callback, t, t1, x);
            t = t1;
            k++;
        }
        if (inputs_cb)
            inputs_cb(t, x, inputs);
        update_history(t, x, inputs);
    }
    else
    {
        uint k = 0;
        double t;
        while (time_cb(k++, t))
        {
            if (inputs_cb)
                inputs_cb(t, x, inputs);
            x.second = stepper_callback(t, x);
            update_history(t, x, inputs);
        }
    }

    return history;
}

// def load_mat_files_as_bus(root, prefix):
//     prefix += "."
//     ret = {}
//     dirpath, dirnames, filenames = next(os.walk(root))
//     for filename in filenames:
//         if not (filename.startswith(prefix) and filename.endswith(".mat")):
//             continue
//         parts = filename.split(".")
//         cur = ret
//         for part in parts[1:-2]:
//             cur = cur.setdefault(part, {})
//         foo = spio.loadmat(os.path.join(dirpath, filename))
//         foo = foo["data"]
//         if foo.shape[0] == 1:
//             cur[parts[-2]] = foo[:, 1:].squeeze()
//         else:
//             cur[parts[-2]] = interp1d(
//                 foo[:, 0], foo[:, 1:].squeeze(), axis=0, assume_sorted=True,
//                 fill_value="extrapolate")

//     return ret

// def interp_bus(dct, t):
//     def traverse(d, r):
//         for k, v in d.items():
//             if isinstance(v, dict):
//                 r._names.append(k)
//                 r.append(blocks.Bus.BusValues([]))
//                 traverse(v, r[-1])
//             elif isinstance(v, interp1d):
//                 foo = v(t)
//                 r._names.append(k)
//                 r.append(foo if foo.shape else float(foo))
//             elif isinstance(v, np.ndarray):
//                 r._names.append(k)
//                 r.append(v if v.shape else float(v))

//     ret = blocks.Bus.BusValues([])
//     traverse(dct, ret)
//     return ret

bool arange(uint k, double& t, double t_init, double t_end, double dt)
{
    t = t_init + k * dt;
    return t <= t_end;
}

}
