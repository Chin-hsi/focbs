#include "lex_astar.h"

void LexAStar::compute_heuristics() {
    my_heuristic.resize(instance.map_size, MAX_TIMESTEP);
    std::queue<pair<int,int>> q;
    my_heuristic[goal_location] = 0;
    q.push({goal_location, 0});
    while (!q.empty()) {
        auto [loc, d] = q.front(); q.pop();
        for (int next : instance.getNeighbors(loc))
            if (my_heuristic[next] > d + 1) {
                my_heuristic[next] = d + 1;
                q.push({next, d + 1});
            }
    }
}

void LexAStar::updatePath(const LexNode* goal, Path& path) {
    const LexNode* curr = goal;
    path.reserve(curr->g_val + 1);
    while (curr != nullptr) {
        path.emplace_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(), path.end());
}

Path LexAStar::findOptimalPath(const CBSNode& node,
                               const ConstraintTable& initial_constraints,
                               const vector<Path*>& paths, int agent, int lowerbound) {
    Path path;
    num_expanded = 0;
    num_generated = 0;

    ConstraintTable ct(initial_constraints);
    ct.insert2CT(node, agent);
    if (ct.constrained(start_location, 0))
        return path;

    ct.insert2CAT(agent, paths);

    auto holding_time = ct.getHoldingTime(goal_location, ct.length_min);
    auto static_timestep = ct.getMaxTimestep() + 1;
    lowerbound = max(holding_time, lowerbound);

    const set<int>* idle_set = ct.idle_starts;

    typedef pairing_heap<LexNode*, compare<LexNode::Compare>> heap_t;
    typedef std::unordered_set<LexNode*, LexNode::Hasher, LexNode::EqNode> hashtable_t;
    heap_t open_list;
    hashtable_t all_nodes;

    int h0 = max(lowerbound, my_heuristic[start_location]);
    int idle0 = (idle_set && idle_set->count(start_location)) ? 1 : 0;
    auto* start_node = new LexNode(start_location, 0, h0, 0, 0, idle0, nullptr);
    num_generated++;
    start_node->open_handle = open_list.push(start_node);
    start_node->in_openlist = true;
    all_nodes.insert(start_node);

    while (!open_list.empty()) {
        auto* curr = open_list.top(); open_list.pop();
        curr->in_openlist = false;
        num_expanded++;

        if (curr->location == goal_location && !curr->wait_at_goal &&
            curr->timestep >= holding_time) {
            updatePath(curr, path);
            break;
        }
        if (curr->timestep >= ct.length_max) continue;

        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);

        for (int next_loc : next_locations) {
            int next_t = curr->timestep + 1;
            if (static_timestep < next_t) {
                if (next_loc == curr->location) continue;
                next_t--;
            }
            if (ct.constrained(next_loc, next_t) ||
                ct.constrained(curr->location, next_loc, next_t))
                continue;

            int next_g = curr->g_val + 1;
            int next_h = max(lowerbound - next_g, my_heuristic[next_loc]);
            if (next_g + next_h > ct.length_max) continue;

            // CAT counts assigned-agent conflicts only.
            int next_cat = curr->cat_conflicts +
                ct.getCATConflicts(curr->location, next_loc, next_t);

            // The start-avoidance penalty is a separate tie-breaking dimension.
            int idle_delta = (idle_set && idle_set->count(next_loc)) ? 1 : 0;
            int next_idle = curr->idle_penalty + idle_delta;

            auto* next = new LexNode(next_loc, next_g, next_h, next_t,
                                     next_cat, next_idle, curr);
            if (next_loc == goal_location && curr->location == goal_location)
                next->wait_at_goal = true;

            auto it = all_nodes.find(next);
            if (it == all_nodes.end()) {
                next->open_handle = open_list.push(next);
                next->in_openlist = true;
                all_nodes.insert(next);
                num_generated++;
                continue;
            }

            auto* existing = *it;
            // Lexicographic dominance: (g, cat, start-avoidance penalty).
            bool dominates =
                next_g < existing->g_val ||
                (next_g == existing->g_val &&
                 next_cat < existing->cat_conflicts) ||
                (next_g == existing->g_val &&
                 next_cat == existing->cat_conflicts &&
                 next_idle < existing->idle_penalty);

            if (dominates) {
                existing->g_val = next_g;
                existing->h_val = next_h;
                existing->cat_conflicts = next_cat;
                existing->idle_penalty = next_idle;
                existing->parent = curr;
                existing->timestep = next_t;
                existing->wait_at_goal = next->wait_at_goal;
                if (existing->in_openlist) {
                    open_list.update(existing->open_handle);
                } else {
                    existing->open_handle = open_list.push(existing);
                    existing->in_openlist = true;
                    num_generated++;
                }
            }
            delete next;
        }
    }

    open_list.clear();
    for (auto* n : all_nodes) delete n;
    return path;
}
