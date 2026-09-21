#pragma once
#include "instance.h"
#include "constraint_table.h"

// Pure A* low-level solver with lexicographic tie-breaking: (f, cat, start-avoidance penalty, h).
// Uses one heap; CAT conflicts and start-avoidance penalties remain separate.

struct LexNode {
    int location;
    int g_val;
    int h_val;
    int timestep;
    int cat_conflicts;   // CAT conflicts between assigned agents only.
    int idle_penalty;    // Start-avoidance penalty.
    LexNode* parent;
    bool in_openlist = false;
    bool wait_at_goal = false;

    struct Compare {
        bool operator()(const LexNode* a, const LexNode* b) const {
            int fa = a->g_val + a->h_val, fb = b->g_val + b->h_val;
            if (fa != fb) return fa > fb;
            if (a->cat_conflicts != b->cat_conflicts)
                return a->cat_conflicts > b->cat_conflicts;
            if (a->idle_penalty != b->idle_penalty)
                return a->idle_penalty > b->idle_penalty;
            return a->h_val > b->h_val;
        }
    };

    LexNode() : location(0), g_val(0), h_val(0), timestep(0),
                cat_conflicts(0), idle_penalty(0), parent(nullptr) {}
    LexNode(int loc, int g, int h, int t, int cat, int start_avoidance_penalty, LexNode* p)
        : location(loc), g_val(g), h_val(h), timestep(t),
          cat_conflicts(cat), idle_penalty(start_avoidance_penalty), parent(p) {}

    int getFVal() const { return g_val + h_val; }

    struct Hasher {
        size_t operator()(const LexNode* n) const {
            return std::hash<int>()(n->location) ^ (std::hash<int>()(n->timestep) << 1);
        }
    };
    struct EqNode {
        bool operator()(const LexNode* a, const LexNode* b) const {
            return a && b && a->location == b->location &&
                   a->timestep == b->timestep &&
                   a->wait_at_goal == b->wait_at_goal;
        }
    };

    typedef pairing_heap<LexNode*, compare<Compare>>::handle_type handle_t;
    handle_t open_handle;
};

class LexAStar {
public:
    uint64_t num_expanded = 0;
    uint64_t num_generated = 0;

    const Instance& instance;
    int start_location;
    int goal_location;
    vector<int> my_heuristic;

    LexAStar(const Instance& inst, int agent)
        : instance(inst),
          start_location(inst.start_locations[agent]),
          goal_location(inst.goal_locations[agent]) {
        compute_heuristics();
    }

    Path findOptimalPath(const CBSNode& node, const ConstraintTable& initial_constraints,
                         const vector<Path*>& paths, int agent, int lowerbound);

private:
    void compute_heuristics();
    void updatePath(const LexNode* goal, Path& path);
};
