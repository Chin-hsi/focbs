#pragma once
#include "cbs_node.h"
#include "conflict_detection.h"
#include "stp_astar.h"
#include "lex_astar.h"
#include "idle_oracle.h"

enum class TENHorizonMode {
    Arrival,
    Input
};

class CBS {
public:
    bool solution_found = false;
    bool timed_out = false;
    bool exact_proven = false;
    int solution_cost = -2;
    double runtime = 0;
    // Wall time spent inside solve().  ``runtime`` is retained as the
    // historical process-CPU timer used by the timeout logic and old CSVs.
    double runtime_wall = 0;

    uint64_t num_HL_expanded = 0;
    uint64_t num_HL_generated = 0;
    uint64_t num_LL_expanded = 0;
    uint64_t num_LL_generated = 0;
    uint64_t num_conflict_scans = 0;
    uint64_t num_conflicts_detected = 0;
    double runtime_conflict_detection = 0;
    int flow_conflict_branches = 0;
    // Exact, online distributions for CT nodes rejected by the flow oracle.
    // Histograms avoid retaining/printing one record per node on large runs.
    uint64_t oracle_infeasible_node_count = 0;
    map<size_t, uint64_t> expanded_depth_histogram;
    map<size_t, uint64_t> oracle_called_depth_histogram;
    map<size_t, uint64_t> oracle_infeasible_depth_histogram;
    map<int, uint64_t> oracle_infeasible_horizon_histogram;
    map<int, uint64_t> oracle_infeasible_blockers_histogram;
    map<int, uint64_t> oracle_infeasible_flow_value_histogram;
    map<int, uint64_t> oracle_infeasible_deficit_histogram;

    // Oracle statistics.
    OracleResult best_oracle;
    vector<Path> best_assigned_paths; // Incumbent assigned paths for Mode X.

    CBS(const Instance& instance, int screen);
    ~CBS();

    bool solve(double time_limit, int cost_lowerbound = 0);
    void clearSearchEngines();
    bool savePaths(const string& fileName) const;
    bool writeSearchTraceJson(const string& fileName, const string& input_label = "") const;

    const vector<Path*>& getPaths() const { return paths; }
    IdleOracle& getOracle() { return *oracle; }
    const Instance& getInstance() const { return instance; }

    // Configuration.
    void setAnytimeInterval(double sec) { anytime_interval = sec; }
    void setEarliestConflict(bool e) { earliest_conflict = e; }
    void setConflictDetection(ConflictDetection d) { conflict_detection = d; }
    void setIncrementalConflicts(bool b) { incremental_conflicts = b; }
    void setFlowConflict(bool fc) { use_flow_conflict = fc; }
    void setExact(bool e) { exact_mode = e; }
    void setTraceEnabled(bool t) { trace_enabled = t; }
    void setBZSSP(bool b);
    void setNoEarlyTerm(bool b);
    void setCostMode(bzssp::CostMode m);
    void setTENHorizonMode(TENHorizonMode mode) { ten_horizon_mode = mode; }
    const char* getTENHorizonModeName() const;
    void setStaticIdle(bool s) {
        if (s && !static_idle) {
            for (auto& ct : initial_constraints)
                for (int loc : instance.idle_locations)
                    ct.insert2CT(loc, 0, MAX_TIMESTEP);
        }
        static_idle = s;
    }
    void setTiebreak(bool t) {
        use_tiebreak = t;
        if (t) {
            for (auto& ct : initial_constraints)
                ct.idle_starts = &idle_start_set;
        }
    }
    void setLexAStar(bool b) {
        if (b && lex_engines.empty()) {
            lex_engines.resize(num_of_agents);
            for (int i = 0; i < num_of_agents; i++)
                lex_engines[i] = new LexAStar(instance, i);
        }
        use_lex_astar = b;
    }

private:
    const Instance& instance;
    int screen;
    int num_of_agents; // assigned agents only
    double time_limit;
    clock_t start;

    bool use_flow_conflict = false;
    bool exact_mode = false;
    bool static_idle = false;
    bool use_tiebreak = false;
    bool earliest_conflict = true;
    ConflictDetection conflict_detection = ConflictDetection::Hash;
    bool incremental_conflicts = true;
    TENHorizonMode ten_horizon_mode = TENHorizonMode::Arrival;
    double anytime_interval = 0; // If positive, report the incumbent every N seconds.
    double last_anytime_report = 0;

    // Unassigned-agent start locations used for tie-breaking.
    set<int> idle_start_set;

    vector<Path> paths_found_initially;
    vector<Path*> paths;
    vector<SpaceTimeAStar*> search_engines;
    vector<LexAStar*> lex_engines;
    bool use_lex_astar = false;
    vector<ConstraintTable> initial_constraints;
    IdleOracle* oracle = nullptr;

    list<CBSNode*> allNodes_table;

    pairing_heap<CBSNode*, compare<CBSNode::compare_node>> open_list;

    // Mode X deduplicates assigned plans by full path encoding.
    std::unordered_set<string> seen_assigned_plans;
    string encodeAssignedPaths() const;

    bool generateRoot();
    bool generateChild(CBSNode* child, CBSNode* parent);
    bool findPathForSingleAgent(CBSNode* node, int ag, int lowerbound = 0);
    void updatePaths(CBSNode* curr);
    void findConflicts(CBSNode& curr);
    shared_ptr<Conflict> chooseConflict(const CBSNode& node) const;
    void prepareNodeForOpen(CBSNode* node);
    void pushNode(CBSNode* node);
    CBSNode* selectNode();
    void releaseNodes();
    bool terminate(CBSNode* curr);
    bool validateSolution() const;
    int getAgentLocation(int agent, size_t timestep) const;

    // High-level secondary keys.
    int computeIdlePenalty() const;
    int computeDistanceToGo(const CBSNode& node) const;

    // Oracle integration.
    int computeArrivalHorizon() const;
    int computeFullAssignedMakespan() const;
    int computeTENHorizon() const;
    bool tryOracle(CBSNode* curr);
    void createFlowConflictBranches(CBSNode* curr, const bzssp::FlowConflict& fc);
    bool constraintExistsInAncestors(CBSNode* node, int agent, int loc1, int loc2, int t, constraint_type type);
    void createExactNoGoodBranches(CBSNode* curr);

    struct NodeTraceInfo {
        int expanded_order = -1;
        string generation_reason = "unknown";
        int generation_blocker_index = -1;
        bool has_generation_blocker = false;
        bzssp::FlowBlocker generation_blocker;

        bool assigned_conflict = false;
        int conflict_a1 = -1;
        int conflict_a2 = -1;
        vector<Constraint> conflict_constraints1;
        vector<Constraint> conflict_constraints2;

        bool oracle_called = false;
        int oracle_horizon = -1;
        OracleResult oracle_result;
        bool flow_branched = false;
        bool heuristic_pruned = false;
        bool exact_nogood_branched = false;
        bool solution_node = false;
    };

    bool trace_enabled = false;
    uint64_t trace_expand_sequence = 0;
    const CBSNode* trace_best_solution_node = nullptr;
    std::unordered_map<const CBSNode*, NodeTraceInfo> trace_nodes;

    void traceSetGenerationReason(CBSNode* node, const string& reason);
    void traceSetFlowBlockerSource(CBSNode* node, int blocker_index, const bzssp::FlowBlocker& blocker);
    void traceMarkExpanded(CBSNode* node);
    void traceRecordChosenConflict(CBSNode* node);
    void traceRecordOracleResult(CBSNode* node, int oracle_horizon,
                                 const OracleResult& result,
                                 bool feasible, bool accepted_solution,
                                 bool flow_branched, bool heuristic_pruned,
                                 bool exact_nogood_branched);
    vector<const Path*> collectNodePaths(const CBSNode* node) const;
    vector<Constraint> collectNodeConstraints(const CBSNode* node) const;
};
