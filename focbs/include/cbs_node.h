#pragma once
#include "common.h"
#include "conflict.h"

class CBSNode {
public:
    list<Constraint> constraints; // Constraints added at this node.
    list<pair<int, Path>> paths;  // Paths replanned at this node.

    int g_val = 0;     // SST of assigned-agent paths.
    int h_val = 0;
    int idle_penalty = 0;       // High-level start-avoidance penalty.
    int distance_to_go = 0;     // High-level conflict-size metric.
    int open_secondary_key = 0; // Current secondary key used by open list.
    int open_tertiary_key = 0;  // Third-level open-list key.
    size_t depth = 0;
    size_t makespan = 0;
    uint64_t time_generated = 0;
    bool conflicts_computed = false;

    list<shared_ptr<Conflict>> conflicts;
    shared_ptr<Conflict> conflict; // Selected conflict.

    CBSNode* parent = nullptr;

    inline int getFVal() const { return g_val + h_val; }
    inline int getNumNewPaths() const { return (int)paths.size(); }

    list<int> getReplannedAgents() const {
        list<int> rst;
        for (const auto& p : paths) rst.push_back(p.first);
        return rst;
    }

    void clear() {
        conflicts.clear();
        conflict = nullptr;
        conflicts_computed = false;
    }

    // Open list order: min f, min secondary, min tertiary, FIFO.
    struct compare_node {
        bool operator()(const CBSNode* n1, const CBSNode* n2) const {
            int f1 = n1->g_val + n1->h_val, f2 = n2->g_val + n2->h_val;
            if (f1 != f2) return f1 > f2;
            if (n1->open_secondary_key != n2->open_secondary_key)
                return n1->open_secondary_key > n2->open_secondary_key;
            if (n1->open_tertiary_key != n2->open_tertiary_key)
                return n1->open_tertiary_key > n2->open_tertiary_key;
            return n1->time_generated > n2->time_generated;
        }
    };

    pairing_heap<CBSNode*, compare<CBSNode::compare_node>>::handle_type open_handle;
};
