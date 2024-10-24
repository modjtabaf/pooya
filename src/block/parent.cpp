/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "parent.hpp"
#include "src/signal/array_signal.hpp"
#include "model.hpp"

namespace pooya
{

std::string Parent::make_signal_name(const std::string& given_name, bool make_new)
{
    pooya_trace("block: " + full_name());
    std::string name(given_name);
    if (name.empty())
    {
        name = "unnamed_signal_" + std::to_string(_unnamed_signal_counter++);
    }
    else
    {
        std::replace(name.begin(), name.end(), ' ', '_');
        std::replace(name.begin(), name.end(), '~', '_');
        std::replace(name.begin(), name.end(), '/', '_');
    }

    std::string sig_name =  _full_name + "~" + name;
    if (!make_new) {return sig_name;}

    // auto& model_ = model_ref();

    // int k = 0;
    std::string postfix = "";
    // while(model_.lookup_signal(sig_name + postfix, true))
    // {
    //     postfix = "_" + std::to_string(k++);
    // }
    return sig_name + postfix;
}

void Parent::_mark_unprocessed()
{
    pooya_trace("block: " + full_name());
    Block::_mark_unprocessed();

#if defined(POOYA_USE_SMART_PTRS)
    for (auto& component: _components)
    {
        component.get()._mark_unprocessed();
    }
#else // defined(POOYA_USE_SMART_PTRS)
    for (auto* component: _components)
    {
        component->_mark_unprocessed();
    }
#endif // defined(POOYA_USE_SMART_PTRS)
}

uint Parent::_process(double t, bool go_deep)
{
    pooya_trace("block: " + full_name());
    uint n_processed = 0;
    if (!_processed)
    {
        _processed = true;
        if (go_deep)
#if defined(POOYA_USE_SMART_PTRS)
            for (auto& component: _components)
            {
                n_processed += component.get()._process(t);
                if (not component.get().processed()) {_processed = false;}
            }
#else // defined(POOYA_USE_SMART_PTRS)
            for (auto* component: _components)
            {
                n_processed += component->_process(t);
                if (not component->processed()) {_processed = false;}
            }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return n_processed;
}

bool Parent::traverse(TraverseCallback cb, uint32_t level, decltype(level) max_level)
{
    pooya_trace("block: " + full_name());
    if (level > max_level) {return true;}

    if (!Block::traverse(cb, level, max_level)) {return false;}

    if (level < max_level)
    {
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& c: _components)
        {
            if (!c.get().traverse(cb, level + 1, max_level)) {return false;}
        }
#else // defined(POOYA_USE_SMART_PTRS)
        for (auto* c: _components)
        {
            if (!c->traverse(cb, level + 1, max_level)) {return false;}
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return true;
}

bool Parent::const_traverse(ConstTraverseCallback cb, uint32_t level, decltype(level) max_level) const
{
    pooya_trace("block: " + full_name());
    if (level > max_level) {return true;}

    if (!Block::const_traverse(cb, level, max_level)) {return false;}

    if (level < max_level)
    {
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& c: _components)
        {
            if (!c.get().const_traverse(cb, level + 1, max_level)) {return false;}
        }
#else // defined(POOYA_USE_SMART_PTRS)
        for (auto* c: _components)
        {
            if (!c->const_traverse(cb, level + 1, max_level)) {return false;}
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return true;
}

// template<typename Iter>
// BusId Parent::create_bus(const std::string& given_name, const BusSpec& spec, Iter begin_, Iter end_)
// {
//     pooya_trace("block: " + full_name());
//     auto size = spec._wires.size();
//     pooya_verify(std::distance(begin_, end_) <= int(size), "Too many entries in the initializer list!");

//     std::vector<LabelSignalId> label_signals;
//     label_signals.reserve(size);
//     for (const auto& wi: spec._wires) {label_signals.push_back({wi.label(), SignalId()});}
//     for (auto it=begin_; it != end_; it++)
//     {
//         auto index = spec.index_of(it->first);
//         pooya_verify(!label_signals.at(index).second, std::string("Duplicate label: ") + it->first);
//         label_signals.at(index).second = it->second;
//     }
//     auto wit = spec._wires.begin();
//     for (auto& ls: label_signals)
//     {
//         if (!ls.second)
//         {
//             std::string name = given_name + "." + wit->label();
//             if (wit->_bus)
//                 {ls.second = create_bus(name, *wit->_bus);}
//             else if (wit->_array_size > 0)
//                 {ls.second = create_array_signal(name, wit->_array_size);}
//             else if (wit->single_value_type() == BusSpec::SingleValueType::Scalar)
//                 {ls.second = create_scalar_signal(name);}
//             else if (wit->single_value_type() == BusSpec::SingleValueType::Int)
//                 {ls.second = create_int_signal(name);}
//             else if (wit->single_value_type() == BusSpec::SingleValueType::Bool)
//                 {ls.second = create_bool_signal(name);}
//             else
//                 {pooya_verify(false, name + ": unknown wire type!");}
//         }
//         wit++;
//     }

//     // return model_ref().register_bus(make_signal_name(given_name, true), spec, label_signals.begin(), label_signals.end());
//     return BusInfo::create_new(make_signal_name(given_name, true), spec, label_signals.begin(), label_signals.end());
// }

// BusId Parent::create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<LabelSignalId>& l)
// {
//     pooya_trace("block: " + full_name());
//     pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");
//     return create_bus(given_name, spec, l.begin(), l.end());
// }

// BusId Parent::create_bus(const std::string& given_name, const BusSpec& spec, const std::initializer_list<SignalId>& l)
// {
//     pooya_trace("block: " + full_name());
//     pooya_verify(l.size() <= spec._wires.size(), "Too many entries in the initializer list!");

//     LabelSignalIdList label_signals;
//     label_signals.reserve(l.size());

//     auto wit = spec._wires.begin();
//     for (const auto& sig: l)
//     {
//         label_signals.push_back({wit->label(), sig});
//         wit++;
//     }

//     return create_bus(given_name, spec, label_signals.begin(), label_signals.end());
// }

bool Parent::add_block(Block& component, const LabelSignals& iports, const LabelSignals& oports)
{
    pooya_trace("block: " + full_name());

    auto make_bus = [&](const LabelSignals& ports) -> BusId
    {
        std::vector<BusSpec::WireInfo> wire_infos;
        LabelSignalIdList wires;
        for (const auto& ls: ports)
        {
            pooya_verify_valid_signal(ls.second);
            if (ls.second->is_scalar())
                {wire_infos.emplace_back(ls.first);}
            else if (ls.second->is_int())
                {wire_infos.emplace_back("i:" + ls.first);}
            else if (ls.second->is_bool())
                {wire_infos.emplace_back("b:" + ls.first);}
            else if (ls.second->is_array())
                {wire_infos.emplace_back(ls.first, ls.second->as_array().size());}
            else if (ls.second->is_bus())
                {wire_infos.emplace_back(ls.first, ls.second->as_bus()._spec);}
            else
                {pooya_verify(false, "unknown signal type!");}
            
            wires.emplace_back(ls.first, ls.second);
        }
        _interface_bus_specs.emplace_back(std::make_unique<BusSpec>(wire_infos.begin(), wire_infos.end()));
        // return create_bus("", *_interface_bus_specs.back(), wires.begin(), wires.end());
        return BusInfo::create_new("", *_interface_bus_specs.back(), wires.begin(), wires.end());
    };

    if (!component.init(*this, make_bus(iports), make_bus(oports))) {return false;}

#if defined(POOYA_USE_SMART_PTRS)
    _components.push_back(component);
#else // defined(POOYA_USE_SMART_PTRS)
    _components.push_back(&component);
#endif // defined(POOYA_USE_SMART_PTRS)
    component.post_init();
    return true;
}

// template<typename T, typename... Ts>
// typename Types<T>::SignalId Parent::create_signal(const std::string& given_name, Ts... args)
// {
//     pooya_trace("block: " + full_name() + ", given name: " + given_name);
//     // return Types<T>::as_signal_id(model_ref().register_signal<T, Ts...>(make_signal_name(given_name, true), args...));
//     return Types<T>::SignalInfo::create_new(make_signal_name(given_name, true), args...);
// }

}
