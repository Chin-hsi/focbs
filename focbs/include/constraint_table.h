#pragma once
#include "common.h"
#include "cbs_node.h"

class ConstraintTable {
public:
    int length_min = 0;
    int length_max = MAX_TIMESTEP;
    size_t num_col;
    size_t map_size;

    int getMaxTimestep() const;
    int getHoldingTime(int location, int earliest_timestep) const;

    bool constrained(size_t loc, int t) const;
    bool constrained(size_t curr_loc, size_t next_loc, int next_t) const;

    // CAT: conflict avoidance table for tie-breaking
    int getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const;
    // CAT conflicts for LexAStar; start-avoidance penalties are tracked separately.
    int getCATConflicts(size_t curr_id, size_t next_id, int next_timestep) const;

    const set<int>* idle_starts = nullptr; // Unassigned-agent start set for start-avoidance penalties.

    ConstraintTable(size_t num_col, size_t map_size) : num_col(num_col), map_size(map_size) {}
    ConstraintTable(const ConstraintTable& other) { copy(other); }

    void copy(const ConstraintTable& other);
    void insert2CT(const CBSNode& node, int agent);
    void insert2CT(const list<Constraint>& constraints, int agent);
    void insert2CT(size_t loc, int t_min, int t_max);
    void insert2CT(size_t from, size_t to, int t_min, int t_max);
    void insert2CAT(int agent, const vector<Path*>& paths);
    void insert2CAT(const Path& path);

private:
    typedef std::unordered_map<size_t, list<pair<int, int>>> CT;
    CT ct;
    int ct_max_timestep = 0;
    typedef vector<vector<bool>> CAT;
    CAT cat;
    int cat_max_timestep = 0;
    vector<int> cat_goals;

    inline size_t getEdgeIndex(size_t from, size_t to) const {
        return (1 + from) * map_size + to;
    }
};
