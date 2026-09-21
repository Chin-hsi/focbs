#pragma once
#include "instance.h"
#include "constraint_table.h"

struct LLNode {
    int location;
    int g_val;
    int h_val = 0;
    LLNode* parent;
    int timestep = 0;
    int num_of_conflicts = 0;  // CAT conflicts only.
    int idle_penalty = 0;      // Start-avoidance penalty.
    bool in_openlist = false;
    bool wait_at_goal = false;
    bool is_goal = false;

    struct compare_node {
        bool operator()(const LLNode* n1, const LLNode* n2) const {
            const int f1 = n1->g_val + n1->h_val;
            const int f2 = n2->g_val + n2->h_val;
            if (f1 != f2) return f1 > f2;
            return n1->h_val > n2->h_val;
        }
    };

    // Focal tie-breaking: min(CAT conflicts, start-avoidance penalty, f, h).
    struct secondary_compare_node {
        bool operator()(const LLNode* n1, const LLNode* n2) const {
            if (n1->num_of_conflicts != n2->num_of_conflicts)
                return n1->num_of_conflicts > n2->num_of_conflicts;
            if (n1->idle_penalty != n2->idle_penalty)
                return n1->idle_penalty > n2->idle_penalty;
            int f1 = n1->g_val + n1->h_val, f2 = n2->g_val + n2->h_val;
            if (f1 != f2) return f1 > f2;
            return n1->h_val > n2->h_val;
        }
    };

    LLNode() : location(0), g_val(0), parent(nullptr) {}
    LLNode(int loc, int g, int h, LLNode* p, int t, int c = 0)
        : location(loc), g_val(g), h_val(h), parent(p), timestep(t), num_of_conflicts(c) {}

    int getFVal() const { return g_val + h_val; }
    void copy(const LLNode& o) {
        location = o.location; g_val = o.g_val; h_val = o.h_val;
        parent = o.parent; timestep = o.timestep;
        num_of_conflicts = o.num_of_conflicts;
        idle_penalty = o.idle_penalty;
        wait_at_goal = o.wait_at_goal; is_goal = o.is_goal;
    }
};

struct AStarNode : public LLNode {
    typedef pairing_heap<AStarNode*, compare<LLNode::compare_node>>::handle_type open_handle_t;
    typedef pairing_heap<AStarNode*, compare<LLNode::secondary_compare_node>>::handle_type focal_handle_t;
    open_handle_t open_handle;
    focal_handle_t focal_handle;

    AStarNode() : LLNode() {}
    AStarNode(int loc, int g, int h, LLNode* p, int t, int c = 0)
        : LLNode(loc, g, h, p, t, c) {}

    struct NodeHasher {
        size_t operator()(const AStarNode* n) const {
            return std::hash<int>()(n->location) ^ (std::hash<int>()(n->timestep) << 1);
        }
    };
    struct eqnode {
        bool operator()(const AStarNode* s1, const AStarNode* s2) const {
            return (s1 == s2) || (s1 && s2 && s1->location == s2->location &&
                                  s1->timestep == s2->timestep &&
                                  s1->wait_at_goal == s2->wait_at_goal);
        }
    };
};

class SpaceTimeAStar {
public:
    uint64_t num_expanded = 0;
    uint64_t num_generated = 0;
    double runtime_build_CT = 0;
    double runtime_build_CAT = 0;

    int start_location;
    int goal_location;
    vector<int> my_heuristic; // Exact BFS distances.
    const Instance& instance;

    Path findOptimalPath(const CBSNode& node, const ConstraintTable& initial_constraints,
                         const vector<Path*>& paths, int agent, int lower_bound);
    pair<Path, int> findSuboptimalPath(const CBSNode& node, const ConstraintTable& initial_constraints,
                                       const vector<Path*>& paths, int agent, int lowerbound, double w);

    SpaceTimeAStar(const Instance& inst, int agent)
        : start_location(inst.start_locations[agent]),
          goal_location(inst.goal_locations[agent]),
          instance(inst) {
        compute_heuristics();
    }

private:
    int min_f_val;
    double w = 1;

    typedef pairing_heap<AStarNode*, compare<LLNode::compare_node>> heap_open_t;
    typedef pairing_heap<AStarNode*, compare<LLNode::secondary_compare_node>> heap_focal_t;
    heap_open_t open_list;
    heap_focal_t focal_list;
    typedef std::unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
    hashtable_t allNodes_table;

    void compute_heuristics();
    void updatePath(const LLNode* goal, Path& path);
    void updateFocalList();
    AStarNode* popNode();
    void pushNode(AStarNode* node);
    void releaseNodes();
};
