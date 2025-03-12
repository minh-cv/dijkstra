#include "dijkstra/map.hpp"
#include <iostream>
#include <cassert>

bool Solver::Cmp::operator()(const StringInt& p1, const StringInt& p2) const {
    return p1.second > p2.second;
}

Solver::Solver(Action::Goal::ConstSharedPtr goal) {
    for (const auto& [first_node, second_node, distance]: goal->map) {
        adj_nodes[first_node].emplace_back(second_node, distance);
        if (result.count(first_node) == 0) {
            result[first_node].node_name = first_node;
            result[first_node].distance = UINT32_MAX;
        }
        if (result.count(second_node) == 0) {
            result[second_node].node_name = second_node;
            result[second_node].distance = UINT32_MAX;
        }
    }
    result[goal->source].node_name = goal->source;
    result[goal->source].distance = 0;
    result[goal->source].path = goal->source;

    current_source = goal->source;
    current_abs_distance = 0;
    iter = adj_nodes[current_source].begin();
}

Solver::Action::Feedback::SharedPtr Solver::continue_solving() {
    while (iter == adj_nodes[current_source].end()) {
        if (pqueue.empty()) {
            is_finished_v = true;
            return nullptr;
        }

        std::tie(current_source, current_abs_distance) = pqueue.top();
        pqueue.pop();
        iter = adj_nodes[current_source].begin();
    }

    auto [current_adj_node, current_rel_distance] = *iter;
    iter++;
    unsigned int new_distance = current_rel_distance + current_abs_distance;

    if (result.count(current_adj_node) == 0) {
        result[current_adj_node].node_name = current_adj_node;
        result[current_adj_node].distance = UINT32_MAX;
    }

    if (new_distance >= result[current_adj_node].distance) {
        return nullptr;
    }

    auto feedback = std::make_shared<Solver::Action::Feedback>();
    result[current_adj_node].distance = new_distance;
    result[current_adj_node].path = result[current_source].path + "->" + current_adj_node;

    pqueue.emplace(current_adj_node, new_distance);
    
    feedback->current_node = result[current_adj_node];
    return feedback;
}

Solver::Action::Result::SharedPtr Solver::get_current_result() const {
    auto action_result = std::make_shared<Action::Result>();
    for (const auto& [_, node_info]: result) {
        action_result->result.push_back(node_info);
    }
    return action_result;
}

bool Solver::is_finished() const {
    return is_finished_v;
}