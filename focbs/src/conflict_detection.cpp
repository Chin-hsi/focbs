#include "conflict_detection.h"
#include <cstdint>

namespace {
int locationAt(const vector<Path*>& paths, int agent, size_t time) {
    const auto& path = *paths[agent];
    return path[min(time, path.size() - 1)].location;
}

uint64_t edgeKey(int from, int to) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(from)) << 32) |
           static_cast<uint32_t>(to);
}

void appendVertex(CBSNode& node, int a1, int a2, int loc, int time) {
    auto conflict = make_shared<Conflict>();
    conflict->vertexConflict(min(a1, a2), max(a1, a2), loc, time);
    node.conflicts.push_back(conflict);
}

void appendEdge(CBSNode& node, int a1, int a2, int from, int to, int time) {
    auto conflict = make_shared<Conflict>();
    // Child 1 always constrains the smaller agent ID, as in CBS-UA.
    if (a1 < a2)
        conflict->edgeConflict(a1, a2, from, to, time);
    else
        conflict->edgeConflict(a2, a1, to, from, time);
    node.conflicts.push_back(conflict);
}

void detectPair(const vector<Path*>& paths, CBSNode& node, int a1, int a2) {
    for (size_t time = 0; time <= node.makespan; ++time) {
        const int loc1 = locationAt(paths, a1, time);
        const int loc2 = locationAt(paths, a2, time);
        if (loc1 == loc2)
            appendVertex(node, a1, a2, loc1, static_cast<int>(time));
        if (time == 0) continue;
        const int from1 = locationAt(paths, a1, time - 1);
        const int from2 = locationAt(paths, a2, time - 1);
        if (from1 != loc1 && from1 == loc2 && loc1 == from2)
            appendEdge(node, a1, a2, from1, loc1, static_cast<int>(time));
    }
}

void detectHash(const vector<Path*>& paths, int map_size, CBSNode& node) {
    vector<vector<int>> occupancy(map_size);
    vector<int> used_locations;
    std::unordered_map<uint64_t, vector<int>> traversals;
    vector<int> previous(paths.size(), -1);

    // Iterate time and then agent ID. Never iterate the unordered_map to emit
    // conflicts: earliest ties must retain the same order as CBS-UA.
    for (size_t time = 0; time <= node.makespan; ++time) {
        for (int loc : used_locations) occupancy[loc].clear();
        used_locations.clear();
        traversals.clear();

        for (int agent = 0; agent < static_cast<int>(paths.size()); ++agent) {
            const int loc = locationAt(paths, agent, time);
            if (occupancy[loc].empty()) used_locations.push_back(loc);
            for (int other : occupancy[loc])
                appendVertex(node, other, agent, loc, static_cast<int>(time));
            occupancy[loc].push_back(agent);

            const int from = previous[agent];
            if (time > 0 && from != loc) {
                const auto reverse = traversals.find(edgeKey(loc, from));
                if (reverse != traversals.end()) {
                    for (int other : reverse->second)
                        appendEdge(node, other, agent, loc, from,
                                   static_cast<int>(time));
                }
                traversals[edgeKey(from, loc)].push_back(agent);
            }
            previous[agent] = loc;
        }
    }
}
} // namespace

void detectAssignedConflicts(const vector<Path*>& paths, int map_size,
                             CBSNode& node, ConflictDetection detection,
                             bool incremental) {
    node.conflicts.clear();
    const auto replanned = node.getReplannedAgents();
    if (incremental && node.parent != nullptr &&
        node.parent->conflicts_computed && !replanned.empty() &&
        node.makespan == node.parent->makespan) {
        vector<bool> changed(paths.size(), false);
        for (int agent : replanned) changed[agent] = true;
        for (const auto& conflict : node.parent->conflicts) {
            if (!changed[conflict->a1] && !changed[conflict->a2])
                node.conflicts.push_back(conflict);
        }
        vector<bool> checked(paths.size(), false);
        for (int agent : replanned) {
            if (checked[agent]) continue;
            for (int other = 0; other < static_cast<int>(paths.size()); ++other) {
                if (other != agent && !checked[other])
                    detectPair(paths, node, agent, other);
            }
            checked[agent] = true;
        }
    } else if (detection == ConflictDetection::Hash) {
        detectHash(paths, map_size, node);
    } else {
        for (int a1 = 0; a1 < static_cast<int>(paths.size()); ++a1)
            for (int a2 = a1 + 1; a2 < static_cast<int>(paths.size()); ++a2)
                detectPair(paths, node, a1, a2);
    }
    node.conflicts_computed = true;
}
