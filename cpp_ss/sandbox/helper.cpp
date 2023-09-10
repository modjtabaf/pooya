
#include <iostream>
#include <fstream>

#include "blocks.hpp"
#include "helper.hpp"

namespace blocks
{

History run(Model& model, TimeCallback time_cb, InputCallback inputs_cb, const NodeIdValues& parameters, Solver stepper)
{
    auto process = [&](double t, NodeIdValues& x) -> uint
    {
        uint n_processed = model._process(t, x, true);
        uint n;
        do
        {
            n = model._process(t, x, false);
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
                    std::cout << "  - i: " << (x.find(p) == x.end() ? "*" : " ") <<  p << "\n";
                for (const auto& p: c->oports())
                    std::cout << "  - o: " << (x.find(p) == x.end() ? "*" : " ") <<  p << "\n";
            }
        }
        return n_processed;
    };

    History history;
    NodeIdValues inputs;

    States states;
    model.get_states(states);
    // std::cout << "53: states\n";
    // auto value = std::get<1>(states).begin();
    // auto deriv = std::get<2>(states).begin();
    // for (auto& state: std::get<0>(states))
    //     std::cout << "  - " << state << " = " << *(value++) << " (d/dt = " << *(deriv++) << ")\n";

    auto stepper_callback = [&](double t, const NodeIdValues& x) -> NodeIdValues
    {
        NodeIdValues y(x);

        y.join(parameters);
        y.join(inputs);
        // std::cout << "65: y\n";
        // for (const auto& v: y)
        //     std::cout << " - " << v.first << " = " << v.second << "\n";

        process(t, y);
        // std::cout << "70: y\n";
        // for (const auto& v: y)
        //     std::cout << " - " << v.first << " = " << v.second << "\n";

        NodeIdValues dx;
        dx.reserve(x.size());
        for (const auto& state: states)
        {
            auto& v = y.at(state.second.second);
            // std::cout << "80: d/dt " << *state << " = " << deriv << "\n";
            dx[state.first] = v;
        }

        // Values ret;
        // ret.reserve(x.size());
        // for (const auto& state: x)
        //     ret.push_back(dx.at(state.first));
        // // std::cout << "81: ret\n";
        // // for (const auto& v: ret)
        // //     std::cout << " - " << v << "\n";

        // return ret;
        return dx;
    };

    NodeIdValues x;
    x.reserve(states.size());
    for (const auto& state: states)
        x.insert_or_assign(state.first, state.second.first);

    auto update_history = [&](double t, const NodeIdValues& x, const NodeIdValues& inputs) -> void
    {
        NodeIdValues y(x);

        y.join(parameters);
        y.join(inputs);

        process(t, y);

        model.step(t, y);

        if (history.empty())
        {
            history.insert_or_assign(0, Value()); // t
            for (const auto& v: y)
                if ((parameters.find(v.first) ==  parameters.end()) && (model.get_node_by_id(v.first)[0] != '-'))
                    history.insert_or_assign(v.first, Value());
        }

        auto& h = history[0]; // t
        auto nrows = h.rows() + 1;
        h.conservativeResize(nrows, NoChange);
        h(nrows - 1, 0) = t;
        uint k = 0;
        for (const auto& v: y)
        {
            if ((parameters.find(v.first) ==  parameters.end()) && (model.get_node_by_id(v.first)[0] != '-'))
            {
                auto& h = history[v.first];
                h.conservativeResize(nrows, NoChange);
                history[v.first].bottomRows<1>() = y.at(v.first);
            }
            k++;
        }
    };

    if (states.size() > 0)
    {
        assert(stepper);
        uint k = 0;
        double t{0}, t1;
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
            auto foo = stepper_callback(t, x);
            for (auto& v: x)
                v.second = foo[v.first];
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
