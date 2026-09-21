#include "stp_astar.h"

void SpaceTimeAStar::compute_heuristics() {
    my_heuristic.resize(instance.map_size, MAX_TIMESTEP);
    std::queue<pair<int,int>> q;
    my_heuristic[goal_location] = 0;
    q.push({goal_location, 0});
    while (!q.empty()) {
        auto [loc, d] = q.front(); q.pop();
        for (int next : instance.getNeighbors(loc)) {
            if (my_heuristic[next] > d + 1) {
                my_heuristic[next] = d + 1;
                q.push({next, d + 1});
            }
        }
    }
}

void SpaceTimeAStar::updatePath(const LLNode* goal, Path& path) {
    const LLNode* curr = goal;
    if (curr->is_goal) curr = curr->parent;
    path.reserve(curr->g_val + 1);
    while (curr != nullptr) {
        path.emplace_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(), path.end());
}

Path SpaceTimeAStar::findOptimalPath(const CBSNode& node, const ConstraintTable& initial_constraints,
                                     const vector<Path*>& paths, int agent, int lowerbound) {
    return findSuboptimalPath(node, initial_constraints, paths, agent, lowerbound, 1).first;
}

pair<Path, int> SpaceTimeAStar::findSuboptimalPath(
    const CBSNode& node, const ConstraintTable& initial_constraints,
    const vector<Path*>& paths, int agent, int lowerbound, double w_) {
    this->w = w_;
    Path path;
    num_expanded = 0;
    num_generated = 0;

    auto t = clock();
    ConstraintTable ct(initial_constraints);
    ct.insert2CT(node, agent);
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
    if (ct.constrained(start_location, 0))
        return {path, 0};

    t = clock();
    ct.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    auto holding_time = ct.getHoldingTime(goal_location, ct.length_min);
    auto static_timestep = ct.getMaxTimestep() + 1;
    lowerbound = max(holding_time, lowerbound);

    auto start = new AStarNode(start_location, 0, max(lowerbound, my_heuristic[start_location]),
                               nullptr, 0, 0);
    start->idle_penalty = (ct.idle_starts && ct.idle_starts->count(start_location)) ? 1 : 0;
    num_generated++;
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);
    min_f_val = start->getFVal();

    while (!open_list.empty()) {
        updateFocalList();
        auto* curr = popNode();

        if (curr->location == goal_location && !curr->wait_at_goal &&
            curr->timestep >= holding_time) {
            updatePath(curr, path);
            break;
        }
        if (curr->timestep >= ct.length_max) continue;

        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations) {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep) {
                if (next_location == curr->location) continue;
                next_timestep--;
            }
            if (ct.constrained(next_location, next_timestep) ||
                ct.constrained(curr->location, next_location, next_timestep))
                continue;

            int next_g = curr->g_val + 1;
            int next_h = max(lowerbound - next_g, my_heuristic[next_location]);
            if (next_g + next_h > ct.length_max) continue;
            // CAT conflicts and start-avoidance penalties are tracked separately.
            int next_conflicts = curr->num_of_conflicts +
                ct.getCATConflicts(curr->location, next_location, next_timestep);
            int next_idle = curr->idle_penalty +
                ((ct.idle_starts && ct.idle_starts->count(next_location)) ? 1 : 0);

            auto next = new AStarNode(next_location, next_g, next_h, curr, next_timestep, next_conflicts);
            next->idle_penalty = next_idle;
            if (next_location == goal_location && curr->location == goal_location)
                next->wait_at_goal = true;

            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end()) {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }

            auto existing = *it;
            // Lexicographic dominance: (f, cat, start-avoidance penalty).
            bool dominates = false;
            int ef = existing->getFVal(), nf = next->getFVal();
            if (nf < ef) dominates = true;
            else if (nf == ef && next->num_of_conflicts < existing->num_of_conflicts) dominates = true;
            else if (nf == ef && next->num_of_conflicts == existing->num_of_conflicts &&
                     next->idle_penalty < existing->idle_penalty) dominates = true;
            if (dominates) {
                if (!existing->in_openlist) {
                    existing->copy(*next);
                    pushNode(existing);
                } else {
                    bool add_to_focal = false, update_in_focal = false, update_open = false;
                    if ((next_g + next_h) <= w * min_f_val) {
                        if (existing->getFVal() > w * min_f_val) add_to_focal = true;
                        else update_in_focal = true;
                    }
                    if (existing->getFVal() > next_g + next_h) update_open = true;
                    existing->copy(*next);
                    if (update_open) open_list.increase(existing->open_handle);
                    if (add_to_focal) existing->focal_handle = focal_list.push(existing);
                    if (update_in_focal) focal_list.update(existing->focal_handle);
                }
            }
            delete next;
        }
    }
    releaseNodes();
    return {path, min_f_val};
}

AStarNode* SpaceTimeAStar::popNode() {
    auto node = focal_list.top(); focal_list.pop();
    open_list.erase(node->open_handle);
    node->in_openlist = false;
    num_expanded++;
    return node;
}

void SpaceTimeAStar::pushNode(AStarNode* node) {
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
    if (node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
}

void SpaceTimeAStar::updateFocalList() {
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val) {
        int new_min = open_head->getFVal();
        for (auto n : open_list)
            if (n->getFVal() > w * min_f_val && n->getFVal() <= w * new_min)
                n->focal_handle = focal_list.push(n);
        min_f_val = new_min;
    }
}

void SpaceTimeAStar::releaseNodes() {
    open_list.clear();
    focal_list.clear();
    for (auto node : allNodes_table) delete node;
    allNodes_table.clear();
}
