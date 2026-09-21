#include "idle_oracle.h"

namespace {
// Declared before TEN temporaries so its destructor also measures their
// deallocation. Otherwise cleanup is incorrectly attributed to CBS.
struct OracleCallTimer {
    IdleOracle& oracle;
    const OracleResult& result;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    ~OracleCallTimer() {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        oracle.oracle_time += elapsed;
        if (result.feasible) oracle.oracle_feasible_time += elapsed;
        else oracle.oracle_infeasible_time += elapsed;
    }
};
long long countIdleMoves(const std::vector<std::vector<std::pair<int, int>>>& idle_paths) {
    long long moves = 0;
    for (const auto& path : idle_paths) {
        for (size_t t = 1; t < path.size(); t++) {
            if (path[t] != path[t - 1])
                moves++;
        }
    }
    return moves;
}
} // namespace

vector<vector<int>> IdleOracle::pathsToCells(const vector<Path*>& paths, int num_assigned) {
    vector<vector<int>> result;
    result.reserve(num_assigned);
    for (int i = 0; i < num_assigned; i++) {
        vector<int> cells;
        cells.reserve(paths[i]->size());
        for (const auto& pe : *paths[i])
            cells.push_back(pe.location);
        result.push_back(std::move(cells));
    }
    return result;
}

bool IdleOracle::solve(const vector<vector<int>>& assigned_paths, int horizon, OracleResult& result) {
    OracleCallTimer timer{*this, result};
    ++oracle_calls;

    result.feasible = false;
    result.smu = 0;
    result.idle_paths.clear();
    result.flowConflict = bzssp::FlowConflict();

    if (instance.num_of_idle == 0) {
        result.feasible = true;
        return true;
    }

    // CBS selects the TEN horizon policy. Use the passed horizon directly.
    if (horizon < 0)
        horizon = 0;

    const auto& idle_cells = instance.idle_locations;

    auto phase_start = std::chrono::steady_clock::now();
    int dimx = instance.num_of_cols;
    int dimy = instance.num_of_rows;
    const auto& grid = instance.getMap();
    // Convert the map encoding to the passable-cell convention used by bzssp.
    vector<bool> passable(grid.size());
    for (size_t i = 0; i < grid.size(); i++)
        passable[i] = !grid[i];

    bzssp::ImplicitInput inp = bzssp::create_implicit_input(
        dimx, dimy, horizon, passable, assigned_paths, idle_cells);
    bzssp::TENInfo ten = bzssp::build_ten_implicit(inp, m_costMode);
    runtime_ten_build += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - phase_start).count();
    if (ten.g.N == 0) {
        ++oracle_infeasible;
        result.flowConflict.flow_value = 0;
        result.flowConflict.deficit = instance.num_of_idle;
        return false;
    }

    phase_start = std::chrono::steady_clock::now();
    bzssp::SSPResult ssp = m_useBZSSP
        ? bzssp::solve_ssp_bz(ten.g, ten.S, ten.T, ten.k,
                               ten.zero_wait_source_arcs,
                               m_noEarlyTerm)
        : bzssp::solve_ssp(ten.g, ten.S, ten.T, ten.k, m_noEarlyTerm);
    runtime_flow_solve += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - phase_start).count();
    bz_zero_total += ssp.bz_zero_count;
    bz_warmstart_total += ssp.bz_zero_count;
    result.flowConflict.flow_value = ssp.augmentations;
    result.flowConflict.deficit = max(0, ten.k - ssp.augmentations);

    if (!ssp.feasible || ssp.augmentations != ten.k) {
        ++oracle_infeasible;
        // Extract flow-conflict blockers from the diagnostic TEN.
        ++diagnostic_calls;
        phase_start = std::chrono::steady_clock::now();
        bzssp::ConflictImplicitInput cinp =
            bzssp::create_conflict_implicit_input(dimx, dimy, horizon, passable, assigned_paths, idle_cells);
        bzssp::ConflictTENInfo cten = bzssp::build_ten_conflict_implicit(cinp);
        runtime_diagnostic_ten_build += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - phase_start).count();
        if (cten.g.N > 0) {
            phase_start = std::chrono::steady_clock::now();
            bzssp::SSPResult ssp2 = bzssp::solve_ssp(cten.g, cten.S, cten.T, cten.k);
            runtime_diagnostic_flow += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - phase_start).count();
            phase_start = std::chrono::steady_clock::now();
            result.flowConflict = bzssp::extract_flow_conflict(cten, ssp2.augmentations);
            runtime_blocker_extraction += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - phase_start).count();
            total_blockers += (int)result.flowConflict.blockers.size();
        }
        return false;
    }

    // Extract unassigned-agent paths.
    phase_start = std::chrono::steady_clock::now();
    bzssp::FlowPaths flow_paths = bzssp::extract_paths(ten);
    runtime_path_extraction += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - phase_start).count();
    if ((int)flow_paths.paths.size() != flow_paths.k) {
        ++oracle_infeasible;
        return false;
    }

    result.feasible = true;
    result.idle_paths = std::move(flow_paths.paths);
    result.smu = countIdleMoves(result.idle_paths);
    return true;
}
