#pragma once
#include "cbs_node.h"

enum class ConflictDetection { Pairwise, Hash };

// Detect vertex and swap conflicts through node.makespan, extending shorter
// paths by waits. Ordering and constraint orientation match CBS-UA, including
// its incremental update order when the planning horizon is unchanged.
void detectAssignedConflicts(const vector<Path*>& paths, int map_size,
                             CBSNode& node, ConflictDetection detection,
                             bool incremental);
