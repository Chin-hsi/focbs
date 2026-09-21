#pragma once
#include "common.h"
#include "instance.h"
#include "bzssp.hpp"

struct OracleResult {
    bool feasible = false;
    long long smu = 0; // Total move cost for unassigned agents.
    std::vector<std::vector<std::pair<int, int>>> idle_paths;
    bzssp::FlowConflict flowConflict;
};

class IdleOracle {
public:
    int oracle_calls = 0;
    int oracle_infeasible = 0;
    double oracle_time = 0;
    double oracle_feasible_time = 0;
    double oracle_infeasible_time = 0;
    // Cumulative steady-clock phase timings across all oracle calls.
    // Primary TEN build includes oracle-input/reservation preparation.
    double runtime_ten_build = 0;
    double runtime_flow_solve = 0;
    double runtime_path_extraction = 0;
    double runtime_diagnostic_ten_build = 0;
    double runtime_diagnostic_flow = 0;
    double runtime_blocker_extraction = 0;
    int diagnostic_calls = 0;
    int total_blockers = 0;
    int bz_zero_total = 0;      // Compatibility counter.
    int bz_warmstart_total = 0; // Preferred counter name.

    IdleOracle(const Instance& inst, bool use_bz_ssp = false,
               bzssp::CostMode cost_mode = bzssp::CostMode::SMU,
               bool no_early_term = false)
        : instance(inst), m_useBZSSP(use_bz_ssp), m_costMode(cost_mode),
          m_noEarlyTerm(no_early_term) {}

    bool useBZSSP() const { return m_useBZSSP; }
    bool noEarlyTerm() const { return m_noEarlyTerm; }

    // assigned_paths stores one cell-id sequence per assigned agent.
    // Returns unassigned-agent paths or a flow conflict.
    bool solve(const vector<vector<int>>& assigned_paths, int horizon, OracleResult& result);

    // Convert CBS Path vectors to cell-id paths.
    static vector<vector<int>> pathsToCells(const vector<Path*>& paths, int num_assigned);

private:
    const Instance& instance;
    bool m_useBZSSP;
    bzssp::CostMode m_costMode;
    bool m_noEarlyTerm;
};
