#include "cbs.h"

#include <sstream>

namespace {

void writeJsonString(std::ostream& os, const string& s) {
    os << '"';
    for (char ch : s) {
        switch (ch) {
            case '\\': os << "\\\\"; break;
            case '"':  os << "\\\""; break;
            case '\n': os << "\\n"; break;
            case '\r': os << "\\r"; break;
            case '\t': os << "\\t"; break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20) {
                    static const char hex[] = "0123456789abcdef";
                    const auto value = static_cast<unsigned char>(ch);
                    os << "\\u00" << hex[value >> 4] << hex[value & 0x0f];
                } else {
                    os << ch;
                }
                break;
        }
    }
    os << '"';
}

const char* constraintTypeName(constraint_type type) {
    switch (type) {
        case constraint_type::VERTEX:    return "vertex";
        case constraint_type::EDGE:      return "edge";
        case constraint_type::LEQLENGTH: return "leqlength";
        case constraint_type::GLENGTH:   return "glength";
        default:                         return "unknown";
    }
}

const char* blockerTypeName(bzssp::FlowBlocker::Type type) {
    return type == bzssp::FlowBlocker::Type::Vertex ? "vertex" : "edge";
}

string formatCell(const Instance& instance, int cell) {
    if (cell < 0) return "cell-1";
    std::ostringstream oss;
    oss << "cell" << cell << "(x=" << instance.getColCoordinate(cell)
        << ",y=" << instance.getRowCoordinate(cell) << ")";
    return oss.str();
}

string formatConstraint(const Instance& instance, const Constraint& c) {
    std::ostringstream oss;
    const int agent = get<0>(c);
    const int loc1 = get<1>(c);
    const int loc2 = get<2>(c);
    const int time = get<3>(c);
    const auto type = get<4>(c);
    oss << "a" << (agent + 1) << " ";
    if (type == constraint_type::VERTEX) {
        oss << "forbid " << formatCell(instance, loc1) << " at t=" << time;
    } else if (type == constraint_type::EDGE) {
        oss << "forbid edge " << formatCell(instance, loc1)
            << "->" << formatCell(instance, loc2) << " at t=" << time;
    } else if (type == constraint_type::LEQLENGTH) {
        oss << "length <= " << time;
    } else if (type == constraint_type::GLENGTH) {
        oss << "length > " << time;
    } else {
        oss << "unknown";
    }
    return oss.str();
}

string formatBlocker(const Instance& instance, const bzssp::FlowBlocker& b) {
    std::ostringstream oss;
    oss << "a" << (b.owner + 1) << " ";
    if (b.type == bzssp::FlowBlocker::Type::Vertex) {
        oss << "vertex " << formatCell(instance, b.cell) << " at t=" << b.time;
    } else {
        oss << "edge " << formatCell(instance, b.from)
            << "->" << formatCell(instance, b.to) << " depart t=" << b.time;
    }
    return oss.str();
}

void writeConstraintJson(std::ostream& os, const Instance& instance, const Constraint& c) {
    os << "{";
    os << "\"agent\":" << get<0>(c) << ",";
    os << "\"label\":";
    writeJsonString(os, "a" + std::to_string(get<0>(c) + 1));
    os << ",";
    os << "\"loc1\":" << get<1>(c) << ",";
    os << "\"loc2\":" << get<2>(c) << ",";
    os << "\"time\":" << get<3>(c) << ",";
    os << "\"type\":";
    writeJsonString(os, constraintTypeName(get<4>(c)));
    os << ",";
    os << "\"repr\":";
    writeJsonString(os, formatConstraint(instance, c));
    os << "}";
}

void writeFlowBlockerJson(std::ostream& os, const Instance& instance, const bzssp::FlowBlocker& b) {
    os << "{";
    os << "\"type\":";
    writeJsonString(os, blockerTypeName(b.type));
    os << ",";
    os << "\"owner\":" << b.owner << ",";
    os << "\"label\":";
    writeJsonString(os, "a" + std::to_string(b.owner + 1));
    os << ",";
    os << "\"time\":" << b.time << ",";
    os << "\"cell\":" << b.cell << ",";
    os << "\"from\":" << b.from << ",";
    os << "\"to\":" << b.to << ",";
    os << "\"repr\":";
    writeJsonString(os, formatBlocker(instance, b));
    os << "}";
}

void writePathCellsJson(std::ostream& os, const Path& path) {
    os << "[";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) os << ",";
        os << path[i].location;
    }
    os << "]";
}

void writeIdlePathJson(std::ostream& os, const Instance& instance,
                       const std::vector<std::pair<int, int>>& path) {
    os << "{";
    os << "\"cells\":[";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) os << ",";
        os << instance.linearizeCoordinate(path[i].second, path[i].first);
    }
    os << "],";
    os << "\"xy\":[";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) os << ",";
        os << "{\"x\":" << path[i].first << ",\"y\":" << path[i].second << "}";
    }
    os << "]";
    os << "}";
}

} // namespace

void CBS::traceSetGenerationReason(CBSNode* node, const string& reason) {
    if (!trace_enabled || node == nullptr) return;
    trace_nodes[node].generation_reason = reason;
}

void CBS::traceSetFlowBlockerSource(CBSNode* node, int blocker_index, const bzssp::FlowBlocker& blocker) {
    if (!trace_enabled || node == nullptr) return;
    auto& info = trace_nodes[node];
    info.generation_reason = "flow_blocker";
    info.generation_blocker_index = blocker_index;
    info.has_generation_blocker = true;
    info.generation_blocker = blocker;
}

void CBS::traceMarkExpanded(CBSNode* node) {
    if (!trace_enabled || node == nullptr) return;
    trace_nodes[node].expanded_order = static_cast<int>(trace_expand_sequence++);
}

void CBS::traceRecordChosenConflict(CBSNode* node) {
    if (!trace_enabled || node == nullptr || node->conflict == nullptr) return;
    auto& info = trace_nodes[node];
    info.assigned_conflict = true;
    info.conflict_a1 = node->conflict->a1;
    info.conflict_a2 = node->conflict->a2;
    info.conflict_constraints1.assign(node->conflict->constraint1.begin(), node->conflict->constraint1.end());
    info.conflict_constraints2.assign(node->conflict->constraint2.begin(), node->conflict->constraint2.end());
}

void CBS::traceRecordOracleResult(CBSNode* node, int oracle_horizon,
                                  const OracleResult& result,
                                  bool feasible, bool accepted_solution,
                                  bool flow_branched, bool heuristic_pruned,
                                  bool exact_nogood_branched) {
    if (!trace_enabled || node == nullptr) return;
    auto& info = trace_nodes[node];
    info.oracle_called = true;
    info.oracle_horizon = oracle_horizon;
    info.oracle_result = result;
    info.oracle_result.feasible = feasible;
    info.flow_branched = flow_branched;
    info.heuristic_pruned = heuristic_pruned;
    info.exact_nogood_branched = exact_nogood_branched;
    info.solution_node = accepted_solution;
}

vector<const Path*> CBS::collectNodePaths(const CBSNode* node) const {
    vector<const Path*> result(num_of_agents, nullptr);
    for (int i = 0; i < num_of_agents; ++i)
        result[i] = &paths_found_initially[i];

    vector<bool> updated(num_of_agents, false);
    while (node != nullptr) {
        for (const auto& p : node->paths) {
            if (!updated[p.first]) {
                result[p.first] = &p.second;
                updated[p.first] = true;
            }
        }
        node = node->parent;
    }
    return result;
}

vector<Constraint> CBS::collectNodeConstraints(const CBSNode* node) const {
    vector<const CBSNode*> chain;
    while (node != nullptr) {
        chain.push_back(node);
        node = node->parent;
    }
    vector<Constraint> result;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        for (const auto& c : (*it)->constraints)
            result.push_back(c);
    }
    return result;
}

bool CBS::writeSearchTraceJson(const string& fileName, const string& input_label) const {
    std::ofstream os(fileName);
    if (!os) return false;

    vector<CBSNode*> nodes(allNodes_table.begin(), allNodes_table.end());
    std::sort(nodes.begin(), nodes.end(),
              [](const CBSNode* a, const CBSNode* b) { return a->time_generated < b->time_generated; });

    std::unordered_map<uint64_t, vector<uint64_t>> children;
    for (const auto* node : nodes) {
        if (node->parent != nullptr)
            children[node->parent->time_generated].push_back(node->time_generated);
    }

    const char* mode =
        exact_mode ? "X" :
        (use_flow_conflict ? "B" : "H");
    auto traceNodeStatus = [](const NodeTraceInfo& info) {
        if (info.expanded_order < 0)
            return string("generated_not_expanded");
        if (info.assigned_conflict)
            return string("assigned_conflict_branched");
        if (!info.oracle_called)
            return string("expanded_other");
        if (info.oracle_result.feasible)
            return info.solution_node ? string("oracle_feasible_solution")
                                      : string("oracle_feasible_nonincumbent");
        if (info.flow_branched)
            return string("oracle_infeasible_flow_branched");
        if (info.exact_nogood_branched)
            return string("oracle_infeasible_exact_nogood");
        if (info.heuristic_pruned)
            return string("oracle_infeasible_pruned");
        return string("oracle_infeasible");
    };

    os << "{\n";

    os << "  \"summary\": {\n";
    os << "    \"mode\": ";
    writeJsonString(os, mode);
    os << ",\n";
    os << "    \"input_label\": ";
    writeJsonString(os, input_label);
    os << ",\n";
    os << "    \"solved\": " << (solution_found ? "true" : "false") << ",\n";
    os << "    \"timed_out\": " << (timed_out ? "true" : "false") << ",\n";
    os << "    \"exact_proven\": " << (exact_proven ? "true" : "false") << ",\n";
    os << "    \"flow_conflict_enabled\": " << (use_flow_conflict ? "true" : "false") << ",\n";
    os << "    \"bz_ssp\": " << (oracle && oracle->useBZSSP() ? "true" : "false") << ",\n";
    os << "    \"ten_horizon_mode\": ";
    writeJsonString(os, getTENHorizonModeName());
    os << ",\n";
    os << "    \"generated\": " << num_HL_generated << ",\n";
    os << "    \"expanded\": " << num_HL_expanded << ",\n";
    os << "    \"solution_cost\": " << solution_cost << ",\n";
    os << "    \"best_solution_node_id\": "
       << (trace_best_solution_node ? static_cast<long long>(trace_best_solution_node->time_generated) : -1) << ",\n";
    os << "    \"runtime\": " << runtime << ",\n";
    os << "    \"oracle_calls\": " << (oracle ? oracle->oracle_calls : 0) << ",\n";
    os << "    \"oracle_infeasible\": " << (oracle ? oracle->oracle_infeasible : 0) << ",\n";
    os << "    \"flow_conflict_branches\": " << flow_conflict_branches << ",\n";
    os << "    \"total_blockers\": " << (oracle ? oracle->total_blockers : 0) << "\n";
    os << "  },\n";

    os << "  \"instance\": {\n";
    os << "    \"cols\": " << instance.num_of_cols << ",\n";
    os << "    \"rows\": " << instance.num_of_rows << ",\n";
    os << "    \"input_horizon\": " << instance.horizon << ",\n";
    os << "    \"assigned\": [\n";
    for (int i = 0; i < instance.num_of_agents; ++i) {
        os << "      {\"agent\":" << i
           << ",\"label\":";
        writeJsonString(os, "a" + std::to_string(i + 1));
        os << ",\"start\":" << instance.start_locations[i]
           << ",\"goal\":" << instance.goal_locations[i]
           << "}";
        if (i + 1 < instance.num_of_agents) os << ",";
        os << "\n";
    }
    os << "    ],\n";
    os << "    \"idle\": [\n";
    for (int i = 0; i < instance.num_of_idle; ++i) {
        os << "      {\"idle\":" << i
           << ",\"label\":";
        writeJsonString(os, "u" + std::to_string(i + 1));
        os << ",\"start\":" << instance.idle_locations[i]
           << "}";
        if (i + 1 < instance.num_of_idle) os << ",";
        os << "\n";
    }
    os << "    ],\n";
    os << "    \"passable\": [";
    const auto& grid = instance.getMap();
    for (size_t i = 0; i < grid.size(); ++i) {
        if (i > 0) os << ",";
        os << (grid[i] ? "false" : "true");
    }
    os << "]\n";
    os << "  },\n";

    os << "  \"nodes\": [\n";
    for (size_t ni = 0; ni < nodes.size(); ++ni) {
        const auto* node = nodes[ni];
        const auto it = trace_nodes.find(node);
        const NodeTraceInfo info = (it != trace_nodes.end()) ? it->second : NodeTraceInfo();
        const auto node_paths = collectNodePaths(node);
        const auto all_constraints = collectNodeConstraints(node);

        os << "    {\n";
        os << "      \"id\": " << node->time_generated << ",\n";
        os << "      \"parent_id\": ";
        if (node->parent != nullptr) os << node->parent->time_generated;
        else os << "null";
        os << ",\n";
        os << "      \"children_ids\": [";
        auto child_it = children.find(node->time_generated);
        if (child_it != children.end()) {
            for (size_t ci = 0; ci < child_it->second.size(); ++ci) {
                if (ci > 0) os << ",";
                os << child_it->second[ci];
            }
        }
        os << "],\n";
        os << "      \"depth\": " << node->depth << ",\n";
        os << "      \"generated_order\": " << node->time_generated << ",\n";
        os << "      \"expanded_order\": " << info.expanded_order << ",\n";
        os << "      \"generation_reason\": ";
        writeJsonString(os, info.generation_reason);
        os << ",\n";
        os << "      \"status\": ";
        writeJsonString(os, traceNodeStatus(info));
        os << ",\n";
        os << "      \"sst\": {\"total\": " << node->g_val << ",\"per_agent\": [";
        for (size_t i = 0; i < node_paths.size(); ++i) {
            if (i > 0) os << ",";
            os << (node_paths[i] ? static_cast<int>(node_paths[i]->size()) - 1 : -1);
        }
        os << "]},\n";
        os << "      \"local_constraints\": [";
        size_t idx = 0;
        for (const auto& c : node->constraints) {
            if (idx++ > 0) os << ",";
            writeConstraintJson(os, instance, c);
        }
        os << "],\n";
        os << "      \"all_constraints\": [";
        for (size_t i = 0; i < all_constraints.size(); ++i) {
            if (i > 0) os << ",";
            writeConstraintJson(os, instance, all_constraints[i]);
        }
        os << "],\n";
        os << "      \"assigned_paths\": [";
        for (size_t ai = 0; ai < node_paths.size(); ++ai) {
            if (ai > 0) os << ",";
            os << "{\"agent\":" << ai << ",\"label\":";
            writeJsonString(os, "a" + std::to_string(ai + 1));
            os << ",\"cells\":";
            writePathCellsJson(os, *node_paths[ai]);
            os << "}";
        }
        os << "],\n";
        os << "      \"selected_assigned_conflict\": ";
        if (!info.assigned_conflict) {
            os << "null";
        } else {
            os << "{";
            os << "\"a1\":" << info.conflict_a1 << ",";
            os << "\"a2\":" << info.conflict_a2 << ",";
            os << "\"constraints1\":[";
            for (size_t i = 0; i < info.conflict_constraints1.size(); ++i) {
                if (i > 0) os << ",";
                writeConstraintJson(os, instance, info.conflict_constraints1[i]);
            }
            os << "],\"constraints2\":[";
            for (size_t i = 0; i < info.conflict_constraints2.size(); ++i) {
                if (i > 0) os << ",";
                writeConstraintJson(os, instance, info.conflict_constraints2[i]);
            }
            os << "]}";
        }
        os << ",\n";
        os << "      \"generation_blocker_index\": " << info.generation_blocker_index << ",\n";
        os << "      \"generation_blocker\": ";
        if (!info.has_generation_blocker) {
            os << "null";
        } else {
            writeFlowBlockerJson(os, instance, info.generation_blocker);
        }
        os << ",\n";
        os << "      \"oracle\": {\n";
        os << "        \"called\": " << (info.oracle_called ? "true" : "false") << ",\n";
        os << "        \"horizon\": " << info.oracle_horizon << ",\n";
        os << "        \"feasible\": " << (info.oracle_called && info.oracle_result.feasible ? "true" : "false") << ",\n";
        os << "        \"solution_node\": " << (info.solution_node ? "true" : "false") << ",\n";
        os << "        \"smu\": " << info.oracle_result.smu << ",\n";
        os << "        \"flow_branched\": " << (info.flow_branched ? "true" : "false") << ",\n";
        os << "        \"heuristic_pruned\": " << (info.heuristic_pruned ? "true" : "false") << ",\n";
        os << "        \"exact_nogood_branched\": " << (info.exact_nogood_branched ? "true" : "false") << ",\n";
        os << "        \"idle_paths\": [";
        for (size_t i = 0; i < info.oracle_result.idle_paths.size(); ++i) {
            if (i > 0) os << ",";
            writeIdlePathJson(os, instance, info.oracle_result.idle_paths[i]);
        }
        os << "],\n";
        os << "        \"flow_conflict\": {\n";
        os << "          \"flow_value\": " << info.oracle_result.flowConflict.flow_value << ",\n";
        os << "          \"deficit\": " << info.oracle_result.flowConflict.deficit << ",\n";
        os << "          \"blockers\": [";
        for (size_t i = 0; i < info.oracle_result.flowConflict.blockers.size(); ++i) {
            if (i > 0) os << ",";
            writeFlowBlockerJson(os, instance, info.oracle_result.flowConflict.blockers[i]);
        }
        os << "]\n";
        os << "        }\n";
        os << "      }\n";
        os << "    }";
        if (ni + 1 < nodes.size()) os << ",";
        os << "\n";
    }
    os << "  ]\n";
    os << "}\n";

    os.close();
    return static_cast<bool>(os);
}
