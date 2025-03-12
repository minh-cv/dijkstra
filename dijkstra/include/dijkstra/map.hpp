#ifndef MAP_HPP
#define MAP_HPP

#include <unordered_map>
#include <vector>
#include <queue>
#include "dijkstra/action/action.hpp"

class Solver {
public:
    using Action = dijkstra::action::Action;
    using NodeInfo = dijkstra::msg::NodeInfo;
    using StringInt = std::pair<std::string, unsigned int>;

    Solver(Action::Goal::ConstSharedPtr goal);
    
    Action::Feedback::SharedPtr continue_solving();
    bool is_finished() const;
    Action::Result::SharedPtr get_current_result() const;

private:
    struct Cmp {
        bool operator()(const Solver::StringInt& p1, const Solver::StringInt& p2) const;
    };

    std::unordered_map<std::string, std::vector<StringInt>> adj_nodes;
    std::priority_queue<StringInt, std::vector<StringInt>, Cmp> pqueue;
    std::unordered_map<std::string, NodeInfo> result;

    //state of function solve
    std::string current_source;
    unsigned int current_abs_distance = 0;
    std::vector<StringInt>::iterator iter;
    bool is_finished_v = false;
};

#endif