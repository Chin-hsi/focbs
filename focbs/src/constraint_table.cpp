#include "constraint_table.h"

int ConstraintTable::getMaxTimestep() const {
    int rst = max(ct_max_timestep, cat_max_timestep);
    rst = max(rst, length_min);
    if (length_max < MAX_TIMESTEP)
        rst = max(rst, length_max);
    return rst;
}

int ConstraintTable::getHoldingTime(int location, int earliest_timestep) const {
    int rst = earliest_timestep;
    auto it = ct.find(location);
    if (it != ct.end())
        for (auto& tr : it->second)
            rst = max(rst, tr.second);
    return rst;
}

void ConstraintTable::insert2CT(size_t from, size_t to, int t_min, int t_max) {
    insert2CT(getEdgeIndex(from, to), t_min, t_max);
}

void ConstraintTable::insert2CT(size_t loc, int t_min, int t_max) {
    ct[loc].emplace_back(t_min, t_max);
    if (t_max < MAX_TIMESTEP && t_max > ct_max_timestep)
        ct_max_timestep = t_max;
    else if (t_max == MAX_TIMESTEP && t_min > ct_max_timestep)
        ct_max_timestep = t_min;
}

void ConstraintTable::insert2CT(const CBSNode& node, int agent) {
    auto curr = &node;
    while (curr->parent != nullptr) {
        insert2CT(curr->constraints, agent);
        curr = curr->parent;
    }
}

void ConstraintTable::insert2CT(const list<Constraint>& constraints, int agent) {
    if (constraints.empty()) return;
    int a, x, y, t;
    constraint_type type;
    tie(a, x, y, t, type) = constraints.front();
    switch (type) {
        case constraint_type::LEQLENGTH:
            if (agent == a)
                length_max = min(length_max, t);
            else
                insert2CT(x, t, MAX_TIMESTEP);
            break;
        case constraint_type::GLENGTH:
            if (a == agent)
                length_min = max(length_min, t + 1);
            break;
        case constraint_type::VERTEX:
            if (a == agent)
                for (const auto& c : constraints) {
                    tie(a, x, y, t, type) = c;
                    insert2CT(x, t, t + 1);
                }
            break;
        case constraint_type::EDGE:
            if (a == agent)
                insert2CT(x, y, t, t + 1);
            break;
        default:
            break;
    }
}

bool ConstraintTable::constrained(size_t loc, int t) const {
    auto it = ct.find(loc);
    if (it == ct.end()) return false;
    for (const auto& c : it->second)
        if (c.first <= t && t < c.second)
            return true;
    return false;
}

bool ConstraintTable::constrained(size_t curr_loc, size_t next_loc, int next_t) const {
    return constrained(getEdgeIndex(curr_loc, next_loc), next_t);
}

void ConstraintTable::copy(const ConstraintTable& other) {
    length_min = other.length_min;
    length_max = other.length_max;
    num_col = other.num_col;
    map_size = other.map_size;
    ct = other.ct;
    ct_max_timestep = other.ct_max_timestep;
    cat = other.cat;
    cat_goals = other.cat_goals;
    cat_max_timestep = other.cat_max_timestep;
    idle_starts = other.idle_starts;
}

void ConstraintTable::insert2CAT(int agent, const vector<Path*>& paths) {
    for (size_t ag = 0; ag < paths.size(); ag++) {
        if ((int)ag == agent || paths[ag] == nullptr) continue;
        insert2CAT(*paths[ag]);
    }
}

void ConstraintTable::insert2CAT(const Path& path) {
    if (cat.empty()) {
        cat.resize(map_size);
        cat_goals.resize(map_size, MAX_TIMESTEP);
    }
    cat_goals[path.back().location] = path.size() - 1;
    for (auto t = (int)path.size() - 1; t >= 0; t--) {
        int loc = path[t].location;
        if ((int)cat[loc].size() <= t)
            cat[loc].resize(t + 1, false);
        cat[loc][t] = true;
    }
    cat_max_timestep = max(cat_max_timestep, (int)path.size() - 1);
}

int ConstraintTable::getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const {
    int rst = 0;
    if (!cat.empty()) {
        if (cat[next_id].size() > (size_t)next_timestep && cat[next_id][next_timestep])
            rst++;
        if (curr_id != next_id && cat[next_id].size() >= (size_t)next_timestep &&
            cat[curr_id].size() > (size_t)next_timestep &&
            cat[next_id][next_timestep - 1] && cat[curr_id][next_timestep])
            rst++;
        if (cat_goals[next_id] < next_timestep)
            rst++;
    }
    // Start-avoidance penalties are stored separately in the low-level search nodes.
    return rst;
}

int ConstraintTable::getCATConflicts(size_t curr_id, size_t next_id, int next_timestep) const {
    int rst = 0;
    if (!cat.empty()) {
        if (cat[next_id].size() > (size_t)next_timestep && cat[next_id][next_timestep])
            rst++;
        if (curr_id != next_id && cat[next_id].size() >= (size_t)next_timestep &&
            cat[curr_id].size() > (size_t)next_timestep &&
            cat[next_id][next_timestep - 1] && cat[curr_id][next_timestep])
            rst++;
        if (cat_goals[next_id] < next_timestep)
            rst++;
    }
    return rst;
}
