#include "cbs.h"

CBS::CBS(const Instance& instance, int screen)
    : instance(instance), screen(screen),
      num_of_agents(instance.num_of_agents) {
    search_engines.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
        search_engines[i] = new SpaceTimeAStar(instance, i);

    // Unassigned-agent start locations.
    for (int loc : instance.idle_locations)
        idle_start_set.insert(loc);

    initial_constraints.resize(num_of_agents, ConstraintTable(instance.num_of_cols, instance.map_size));

    oracle = new IdleOracle(instance);
}

CBS::~CBS() {
    clearSearchEngines();
    releaseNodes();
}

void CBS::clearSearchEngines() {
    for (auto e : search_engines) delete e;
    search_engines.clear();
    for (auto e : lex_engines) delete e;
    lex_engines.clear();
    delete oracle; oracle = nullptr;
}

void CBS::setBZSSP(bool b) {
    bool net = oracle ? oracle->noEarlyTerm() : false;
    delete oracle;
    oracle = new IdleOracle(instance, b, bzssp::CostMode::SMU, net);
}

void CBS::setNoEarlyTerm(bool b) {
    bool bz = oracle ? oracle->useBZSSP() : false;
    delete oracle;
    oracle = new IdleOracle(instance, bz, bzssp::CostMode::SMU, b);
}

void CBS::setCostMode(bzssp::CostMode m) {
    bool bz = oracle ? oracle->useBZSSP() : false;
    bool net = oracle ? oracle->noEarlyTerm() : false;
    delete oracle;
    oracle = new IdleOracle(instance, bz, m, net);
}

const char* CBS::getTENHorizonModeName() const {
    switch (ten_horizon_mode) {
        case TENHorizonMode::Arrival: return "arrival";
        case TENHorizonMode::Input: return "input";
        default: return "unknown";
    }
}

int CBS::computeFullAssignedMakespan() const {
    int horizon = 0;
    for (int i = 0; i < num_of_agents; i++) {
        if (paths[i] == nullptr || paths[i]->empty())
            continue;
        horizon = max(horizon, (int)paths[i]->size() - 1);
    }
    return horizon;
}

int CBS::computeArrivalHorizon() const {
    int horizon = 0;
    for (int i = 0; i < num_of_agents; i++) {
        if (paths[i] == nullptr || paths[i]->empty())
            continue;
        const auto& path = *paths[i];
        int arrival = (int)path.size() - 1;
        const int goal = instance.goal_locations[i];
        if (path.back().location == goal) {
            while (arrival > 0 && path[arrival - 1].location == goal)
                arrival--;
        }
        horizon = max(horizon, arrival);
    }
    return horizon;
}

int CBS::computeTENHorizon() const {
    switch (ten_horizon_mode) {
        case TENHorizonMode::Arrival:
            return computeArrivalHorizon();
        case TENHorizonMode::Input:
            return max(max(0, instance.horizon), computeFullAssignedMakespan());
        default:
            return computeArrivalHorizon();
    }
}


bool CBS::solve(double _time_limit, int cost_lowerbound) {
    (void)cost_lowerbound;
    this->time_limit = _time_limit;
    const auto wall_start = std::chrono::steady_clock::now();
    start = clock();
    solution_found = false;
    timed_out = false;
    exact_proven = false;
    solution_cost = -2;
    runtime = 0;
    runtime_wall = 0;
    num_conflict_scans = 0;
    num_conflicts_detected = 0;
    runtime_conflict_detection = 0;
    best_oracle = OracleResult();
    best_assigned_paths.clear();
    seen_assigned_plans.clear();
    trace_expand_sequence = 0;
    trace_best_solution_node = nullptr;
    trace_nodes.clear();
    oracle_infeasible_node_count = 0;
    expanded_depth_histogram.clear();
    oracle_called_depth_histogram.clear();
    oracle_infeasible_depth_histogram.clear();
    oracle_infeasible_horizon_histogram.clear();
    oracle_infeasible_blockers_histogram.clear();
    oracle_infeasible_flow_value_histogram.clear();
    oracle_infeasible_deficit_histogram.clear();
    if (!generateRoot()) {
        runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
        runtime_wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - wall_start).count();
        return false;
    }

    while (!open_list.empty()) {
        // Report incumbents periodically in anytime mode.
        if (anytime_interval > 0 && solution_found) {
            double elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
            if (elapsed - last_anytime_report >= anytime_interval) {
                last_anytime_report = elapsed;
                cerr << "ANYTIME: t=" << elapsed
                     << " sst=" << solution_cost
                     << " smu=" << best_oracle.smu << endl;
            }
        }
        auto curr = selectNode();
        if (terminate(curr)) break;

        // Nodes are queued with conflict summaries and high-level ordering keys ready.
        if (!curr->conflicts_computed)
            prepareNodeForOpen(curr);

        curr->conflict = chooseConflict(*curr);
        traceRecordChosenConflict(curr);

        if (curr->conflict == nullptr) {
            // No assigned-agent conflict remains, so call the oracle.
            if (tryOracle(curr))
                break; // Found a complete solution.
            continue;  // Oracle infeasibility was handled by pruning or branching.
        }

        // Standard two-way CBS split.
        auto child1 = new CBSNode();
        auto child2 = new CBSNode();
        child1->constraints = curr->conflict->constraint1;
        child2->constraints = curr->conflict->constraint2;
        traceSetGenerationReason(child1, "standard_conflict");
        traceSetGenerationReason(child2, "standard_conflict");

        bool succ1 = generateChild(child1, curr);
        updatePaths(curr); // restore before second child
        bool succ2 = generateChild(child2, curr);

        if (succ1) pushNode(child1); else delete child1;
        if (succ2) pushNode(child2); else delete child2;
    }

    runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
    runtime_wall = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wall_start).count();
    if (exact_mode && solution_found && !timed_out && !exact_proven)
        exact_proven = true;
    // Restore incumbent paths for savePaths/getPaths in Mode X.
    if (exact_mode && solution_found && !best_assigned_paths.empty()) {
        for (int i = 0; i < num_of_agents; i++)
            paths_found_initially[i] = best_assigned_paths[i];
        for (int i = 0; i < num_of_agents; i++)
            paths[i] = &paths_found_initially[i];
    }
    return solution_found;
}

string CBS::encodeAssignedPaths() const {
    string key;
    for (int i = 0; i < num_of_agents; i++) {
        key += "a";
        key += std::to_string(i);
        key += ":";
        for (const auto& pe : *paths[i]) {
            key += std::to_string(pe.location);
            key += ",";
        }
        key += ";";
    }
    return key;
}

bool CBS::tryOracle(CBSNode* curr) {
    if (static_idle || instance.num_of_idle == 0) {
        solution_found = true;
        solution_cost = curr->g_val;
        return true;
    }

    // Mode X skips assigned plans that were already evaluated.
    if (exact_mode) {
        string key = encodeAssignedPaths();
        if (!seen_assigned_plans.insert(key).second) {
            createExactNoGoodBranches(curr);
            return false;
        }
    }

    auto assigned_paths = IdleOracle::pathsToCells(paths, num_of_agents);
    int oracle_horizon = computeTENHorizon();
    OracleResult result;
    ++oracle_called_depth_histogram[curr->depth];
    bool feasible = oracle->solve(assigned_paths, oracle_horizon, result);

    if (feasible) {
        bool accepted_solution = !exact_mode || !solution_found || result.smu < best_oracle.smu;
        if (accepted_solution) {
            solution_found = true;
            solution_cost = curr->g_val;
            best_oracle = std::move(result);
            trace_best_solution_node = curr;
            if (exact_mode) {
                best_assigned_paths.resize(num_of_agents);
                for (int i = 0; i < num_of_agents; i++)
                    best_assigned_paths[i] = *paths[i];
            }
        }
        traceRecordOracleResult(curr, oracle_horizon, accepted_solution ? best_oracle : result,
                                true, accepted_solution, false, false, false);
        if (!exact_mode) return true;
        // Mode X continues through nodes with the same SST.
        createExactNoGoodBranches(curr);
        return false;
    }

    ++oracle_infeasible_node_count;
    ++oracle_infeasible_depth_histogram[curr->depth];
    ++oracle_infeasible_horizon_histogram[oracle_horizon];
    ++oracle_infeasible_blockers_histogram[
        static_cast<int>(result.flowConflict.blockers.size())];
    ++oracle_infeasible_flow_value_histogram[result.flowConflict.flow_value];
    ++oracle_infeasible_deficit_histogram[result.flowConflict.deficit];

    if (use_flow_conflict && !result.flowConflict.blockers.empty()) {
        traceRecordOracleResult(curr, oracle_horizon, result, false, false, true, false, false);
        createFlowConflictBranches(curr, result.flowConflict);
    } else if (exact_mode) {
        // Mode X adds no-good constraints when no flow-conflict branch is available.
        traceRecordOracleResult(curr, oracle_horizon, result, false, false, false, false, true);
        createExactNoGoodBranches(curr);
    } else {
        // Mode H prunes oracle-infeasible nodes.
        traceRecordOracleResult(curr, oracle_horizon, result, false, false, false, true, false);
    }
    return false;
}

// Check whether a constraint already appears on the ancestor chain.
bool CBS::constraintExistsInAncestors(CBSNode* node, int agent, int loc1, int loc2, int t, constraint_type type) {
    CBSNode* p = node;
    while (p != nullptr) {
        for (auto& c : p->constraints) {
            if (get<0>(c) == agent && get<1>(c) == loc1 &&
                get<2>(c) == loc2 && get<3>(c) == t && get<4>(c) == type)
                return true;
        }
        p = p->parent;
    }
    return false;
}

void CBS::createFlowConflictBranches(CBSNode* curr, const bzssp::FlowConflict& fc) {
    for (size_t bi = 0; bi < fc.blockers.size(); ++bi) {
        const auto& b = fc.blockers[bi];
        if (b.owner < 0 || b.owner >= num_of_agents) continue;
        if (b.type == bzssp::FlowBlocker::Type::Vertex && b.time == 0) continue;

        // BZ-SSP edge blockers use departure time t; CBS edge constraints use arrival time t+1.
        int ct_time = (b.type == bzssp::FlowBlocker::Type::Edge) ? b.time + 1 : b.time;

        // Skip duplicate constraints already present on the ancestor chain.
        int loc1 = (b.type == bzssp::FlowBlocker::Type::Vertex) ? b.cell : b.from;
        int loc2 = (b.type == bzssp::FlowBlocker::Type::Vertex) ? -1 : b.to;
        constraint_type ctype = (b.type == bzssp::FlowBlocker::Type::Vertex)
            ? constraint_type::VERTEX : constraint_type::EDGE;
        if (constraintExistsInAncestors(curr, b.owner, loc1, loc2, ct_time, ctype))
            continue;

        updatePaths(curr);
        auto child = new CBSNode();
        traceSetGenerationReason(child, "flow_blocker");
        traceSetFlowBlockerSource(child, (int)bi, b);
        if (b.type == bzssp::FlowBlocker::Type::Vertex) {
            child->constraints.emplace_back(b.owner, b.cell, -1, b.time, constraint_type::VERTEX);
        } else {
            child->constraints.emplace_back(b.owner, b.from, b.to, ct_time, constraint_type::EDGE);
        }
        if (generateChild(child, curr)) {
            pushNode(child);
            flow_conflict_branches++;
        } else
            delete child;
    }
}

void CBS::createExactNoGoodBranches(CBSNode* curr) {
    // Snapshot paths before generating children because generateChild mutates paths[].
    updatePaths(curr);
    vector<Path> snapshot(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
        snapshot[i] = *paths[i];

    for (int i = 0; i < num_of_agents; i++) {
        int goal = instance.goal_locations[i];
        // Find the final arrival time after which the agent stays at the goal.
        int final_arrival = (int)snapshot[i].size();
        for (int t2 = (int)snapshot[i].size() - 1; t2 >= 0; t2--) {
            if (snapshot[i].at(t2).location == goal)
                final_arrival = t2;
            else
                break;
        }
        for (size_t t = 1; t < snapshot[i].size(); t++) {
            int loc = snapshot[i].at(t).location;
            // Skip only permanent goal waits; transient goal visits can still branch.
            if (loc == goal && (int)t >= final_arrival) continue;
            updatePaths(curr);
            auto child = new CBSNode();
            traceSetGenerationReason(child, "exact_no_good");
            child->constraints.emplace_back(i, loc, -1, (int)t, constraint_type::VERTEX);
            if (generateChild(child, curr))
                pushNode(child);
            else
                delete child;
        }
    }
}


bool CBS::generateRoot() {
    auto root = new CBSNode();
    traceSetGenerationReason(root, "root");
    root->g_val = 0;
    root->depth = 0;
    paths.resize(num_of_agents, nullptr);
    paths_found_initially.resize(num_of_agents);

    for (int i = 0; i < num_of_agents; i++) {
        if (use_lex_astar) {
            paths_found_initially[i] = lex_engines[i]->findOptimalPath(
                *root, initial_constraints[i], paths, i, 0);
        } else {
            paths_found_initially[i] = search_engines[i]->findOptimalPath(
                *root, initial_constraints[i], paths, i, 0);
        }
        if (paths_found_initially[i].empty()) {
            if (screen > 0) cerr << "No path for agent " << i << endl;
            delete root;
            return false;
        }
        paths[i] = &paths_found_initially[i];
        root->makespan = max(root->makespan, paths_found_initially[i].size() - 1);
        root->g_val += (int)paths_found_initially[i].size() - 1;
        auto ll_exp = use_lex_astar ? lex_engines[i]->num_expanded : search_engines[i]->num_expanded;
        auto ll_gen = use_lex_astar ? lex_engines[i]->num_generated : search_engines[i]->num_generated;
        num_LL_expanded += ll_exp;
        num_LL_generated += ll_gen;
    }

    prepareNodeForOpen(root);
    root->time_generated = num_HL_generated++;
    pushNode(root);

    if (screen > 0)
        cout << "Root: cost=" << root->g_val << ", LL=" << num_LL_expanded << endl;
    return true;
}

bool CBS::generateChild(CBSNode* child, CBSNode* parent) {
    child->parent = parent;
    child->g_val = parent->g_val;
    child->makespan = parent->makespan;
    child->depth = parent->depth + 1;

    // Determine which agents need replanning.
    int a = get<0>(child->constraints.front());
    assert(a >= 0 && a < num_of_agents);

    int lowerbound = (int)paths[a]->size() - 1;
    if (!findPathForSingleAgent(child, a, lowerbound)) {
        trace_nodes.erase(child);
        return false;
    }

    prepareNodeForOpen(child);
    child->time_generated = num_HL_generated++;
    return true;
}

bool CBS::findPathForSingleAgent(CBSNode* node, int ag, int lowerbound) {
    Path new_path;
    if (use_lex_astar) {
        new_path = lex_engines[ag]->findOptimalPath(
            *node, initial_constraints[ag], paths, ag, lowerbound);
        num_LL_expanded += lex_engines[ag]->num_expanded;
        num_LL_generated += lex_engines[ag]->num_generated;
    } else {
        new_path = search_engines[ag]->findOptimalPath(
            *node, initial_constraints[ag], paths, ag, lowerbound);
        num_LL_expanded += search_engines[ag]->num_expanded;
        num_LL_generated += search_engines[ag]->num_generated;
    }

    if (!new_path.empty()) {
        node->paths.emplace_back(ag, new_path);
        node->g_val = node->g_val - (int)paths[ag]->size() + (int)new_path.size();
        paths[ag] = &node->paths.back().second;
        node->makespan = max(node->makespan, new_path.size() - 1);
        return true;
    }
    return false;
}


void CBS::findConflicts(CBSNode& curr) {
    const auto started = std::chrono::steady_clock::now();
    detectAssignedConflicts(paths, instance.map_size, curr,
                            conflict_detection, incremental_conflicts);
    ++num_conflict_scans;
    num_conflicts_detected += curr.conflicts.size();
    runtime_conflict_detection += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
}

void CBS::prepareNodeForOpen(CBSNode* node) {
    updatePaths(node);
    node->clear();
    node->makespan = computeFullAssignedMakespan();
    findConflicts(*node);
    node->conflicts_computed = true;
    node->distance_to_go = computeDistanceToGo(*node);
    node->idle_penalty = computeIdlePenalty();
    if (use_tiebreak) {
        node->open_secondary_key = node->distance_to_go;
        node->open_tertiary_key = node->idle_penalty;
    } else {
        node->open_secondary_key = node->distance_to_go;
        node->open_tertiary_key = 0;
    }
}

shared_ptr<Conflict> CBS::chooseConflict(const CBSNode& node) const {
    if (node.conflicts.empty()) return nullptr;
    if (earliest_conflict) {
        shared_ptr<Conflict> best = nullptr;
        int best_t = INT_MAX;
        for (auto& c : node.conflicts) {
            int t = std::get<3>(c->constraint1.front());
            if (t < best_t) { best_t = t; best = c; }
        }
        return best;
    }
    auto it = node.conflicts.begin();
    int idx = rand() % node.conflicts.size();
    std::advance(it, idx);
    return *it;
}


void CBS::updatePaths(CBSNode* curr) {
    for (int i = 0; i < num_of_agents; i++)
        paths[i] = &paths_found_initially[i];
    vector<bool> updated(num_of_agents, false);
    while (curr != nullptr) {
        for (auto& p : curr->paths) {
            if (!updated[p.first]) {
                paths[p.first] = &(p.second);
                updated[p.first] = true;
            }
        }
        curr = curr->parent;
    }
}

void CBS::pushNode(CBSNode* node) {
    if (!node->conflicts_computed)
        prepareNodeForOpen(node);
    allNodes_table.push_back(node);
    node->open_handle = open_list.push(node);
}

CBSNode* CBS::selectNode() {
    auto node = open_list.top(); open_list.pop();
    updatePaths(node);
    num_HL_expanded++;
    ++expanded_depth_histogram[node->depth];
    traceMarkExpanded(node);
    if (screen > 1)
        cout << "Expand node " << node->time_generated << " (f=" << node->getFVal()
             << ", #conflicts=" << node->conflicts.size() << ")" << endl;
    return node;
}

bool CBS::terminate(CBSNode* curr) {
    // Mode X proof takes priority over timeout at the end of an SST cost layer.
    if (exact_mode && solution_found && curr->g_val > solution_cost) {
        exact_proven = true;
        if (screen > 0) cout << "Exact: all nodes with cost=" << solution_cost << " explored" << endl;
        return true;
    }
    double elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
    if (elapsed > time_limit) {
        timed_out = true;
        if (screen > 0) cout << "Timeout" << endl;
        return true;
    }
    // CBS constraints can require paths longer than any fixed graph-size bound.
    // Termination relies on the timeout and Mode X cost-layer completion.
    return false;
}

void CBS::releaseNodes() {
    open_list.clear();
    for (auto node : allNodes_table) delete node;
    allNodes_table.clear();
}

bool CBS::validateSolution() const {
    for (int a1 = 0; a1 < num_of_agents; a1++) {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++) {
            int len = (int)max(paths[a1]->size(), paths[a2]->size());
            for (int t = 0; t < len; t++) {
                if (getAgentLocation(a1, t) == getAgentLocation(a2, t)) {
                    cerr << "Vertex conflict: a" << a1 << " a" << a2 << " t=" << t << endl;
                    return false;
                }
                if (t > 0 &&
                    getAgentLocation(a1, t-1) == getAgentLocation(a2, t) &&
                    getAgentLocation(a1, t) == getAgentLocation(a2, t-1)) {
                    cerr << "Edge conflict: a" << a1 << " a" << a2 << " t=" << t << endl;
                    return false;
                }
            }
        }
    }
    return true;
}

int CBS::getAgentLocation(int agent, size_t timestep) const {
    if (timestep < paths[agent]->size())
        return paths[agent]->at(timestep).location;
    return paths[agent]->back().location;
}

int CBS::computeIdlePenalty() const {
    if (!use_tiebreak || idle_start_set.empty()) return 0;
    int penalty = 0;
    for (int i = 0; i < num_of_agents; i++)
        for (const auto& pe : *paths[i])
            if (idle_start_set.count(pe.location))
                penalty++;
    return penalty;
}

int CBS::computeDistanceToGo(const CBSNode& node) const {
    set<pair<int, int>> conflicting_pairs;
    for (const auto& c : node.conflicts)
        conflicting_pairs.emplace(min(c->a1, c->a2), max(c->a1, c->a2));
    return (int)conflicting_pairs.size();
}

bool CBS::savePaths(const string& fileName) const {
    ofstream out(fileName);
    if (!out) return false;
    for (int i = 0; i < num_of_agents; i++) {
        out << "Agent " << i << ":";
        for (const auto& pe : *paths[i])
            out << " " << pe.location;
        out << endl;
    }
    out.close();
    return static_cast<bool>(out);
}
