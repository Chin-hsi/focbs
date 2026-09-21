#include "cbs.h"
#include <boost/program_options.hpp>
#include <cctype>
#include <cmath>
#include <iomanip>

namespace {
int countMoves(const Path& path) {
    int moves = 0;
    for (size_t t = 1; t < path.size(); t++) {
        if (path[t].location != path[t - 1].location)
            moves++;
    }
    return moves;
}

int countMoves(const std::vector<std::pair<int, int>>& path) {
    int moves = 0;
    for (size_t t = 1; t < path.size(); t++) {
        if (path[t] != path[t - 1])
            moves++;
    }
    return moves;
}

int computeAssignedSST(const Instance& instance, const vector<Path*>& paths) {
    int sst = 0;
    for (int i = 0; i < instance.num_of_agents; i++) {
        if (paths[i] == nullptr || paths[i]->empty())
            continue;
        const auto& path = *paths[i];
        int arrival = static_cast<int>(path.size()) - 1;
        const int goal = instance.goal_locations[i];
        if (path.back().location == goal) {
            while (arrival > 0 && path[arrival - 1].location == goal)
                arrival--;
        }
        sst += arrival;
    }
    return sst;
}

int computeAssignedFuel(const vector<Path*>& paths) {
    int fuel = 0;
    for (auto* path : paths) {
        if (path != nullptr)
            fuel += countMoves(*path);
    }
    return fuel;
}

int computeIdleSMU(const std::vector<std::vector<std::pair<int, int>>>& idle_paths) {
    int smu = 0;
    for (const auto& path : idle_paths)
        smu += countMoves(path);
    return smu;
}

int computeNUA(const std::vector<std::vector<std::pair<int, int>>>& idle_paths) {
    int nua = 0;
    for (const auto& path : idle_paths) {
        if (countMoves(path) > 0)
            nua++;
    }
    return nua;
}

template <typename Key>
void printHistogram(const string& name, const map<Key, uint64_t>& histogram) {
    cout << "  " << name << ": ";
    if (histogram.empty()) {
        cout << "none" << endl;
        return;
    }
    bool first = true;
    for (const auto& item : histogram) {
        if (!first) cout << ";";
        first = false;
        cout << item.first << ":" << item.second;
    }
    cout << endl;
}

void printOracleInfeasibleNodeDistribution(const CBS& cbs) {
    printHistogram("expandedDepthHistogram", cbs.expanded_depth_histogram);
    printHistogram("oracleCalledDepthHistogram", cbs.oracle_called_depth_histogram);
    cout << "  oracleInfeasibleNodeCount: "
         << cbs.oracle_infeasible_node_count << endl;
    printHistogram("oracleInfeasibleDepthHistogram",
                   cbs.oracle_infeasible_depth_histogram);
    printHistogram("oracleInfeasibleHorizonHistogram",
                   cbs.oracle_infeasible_horizon_histogram);
    printHistogram("oracleInfeasibleBlockersHistogram",
                   cbs.oracle_infeasible_blockers_histogram);
    printHistogram("oracleInfeasibleFlowValueHistogram",
                   cbs.oracle_infeasible_flow_value_histogram);
    printHistogram("oracleInfeasibleDeficitHistogram",
                   cbs.oracle_infeasible_deficit_histogram);
}

struct ModeConfig {
    string internal_mode = "B";
    bool flow_conflict = true;
    bool use_bz_ssp = true;
    bool use_tiebreak = true;
};

bool configureNamedMode(string requested, ModeConfig& cfg) {
    for (char& c : requested)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (requested == "FOCBS") requested = "B-BZ-TB";
    else if (requested == "FOCBS-NOTB") requested = "B-BZ";
    else if (requested == "FOCBS-NOBZ") requested = "B-TB";
    else if (requested == "FOCBS-H") requested = "H-BZ";

    if (requested == "B-BZ") {
        cfg.internal_mode = "B";
        cfg.flow_conflict = true;
        cfg.use_bz_ssp = true;
        cfg.use_tiebreak = false;
        return true;
    }
    if (requested == "B-BZ-TB" || requested == "B-TB") {
        cfg.internal_mode = "B";
        cfg.flow_conflict = true;
        cfg.use_bz_ssp = requested == "B-BZ-TB";
        cfg.use_tiebreak = true;
        return true;
    }
    if (requested == "H-BZ") {
        cfg.internal_mode = "H";
        cfg.flow_conflict = false;
        cfg.use_bz_ssp = true;
        cfg.use_tiebreak = false;
        return true;
    }
    if (requested == "H-BZ-TB") {
        cfg.internal_mode = "H";
        cfg.flow_conflict = false;
        cfg.use_bz_ssp = true;
        cfg.use_tiebreak = true;
        return true;
    }
    return false;
}
} // namespace

int main(int argc, char** argv) {
    namespace po = boost::program_options;
#ifdef FOCBS_REVIEW_CLI
    po::options_description input_options("Input, output, and logging");
    input_options.add_options()
        ("help", "help")
        ("input,i", po::value<string>(), ".mapfua input file")
        ("map,m", po::value<string>(), "MovingAI .map file")
        ("agents,a", po::value<string>(), "MovingAI .scen file")
        ("agentNum,k", po::value<int>()->default_value(0), "number of agents from .scen")
        ("output,o", po::value<string>(), "output file for paths")
        ("no-schedule", po::bool_switch(),
         "suppress YAML schedule output (for benchmarks)")
        ("timeout,t", po::value<double>()->default_value(60), "timeout (seconds)")
        ("screen,s", po::value<int>()->default_value(1), "screen level (0/1/2)")
    ;

    po::options_description algorithm_options("Algorithm options");
    algorithm_options.add_options()
        ("mode", po::value<string>()->default_value("FOCBS"),
         "FOCBS=B-BZ-TB, FOCBS-noTB=B-BZ, FOCBS-noBZ=B-TB, FOCBS-H=H-BZ; also accepts H-BZ-TB")
    ;

    po::options_description desc("FOCBS");
    desc.add(input_options).add(algorithm_options);
#else
    po::options_description input_options("Input, output, and logging");
    input_options.add_options()
        ("help", "help")
        ("input,i", po::value<string>(), ".mapfua input file")
        ("map,m", po::value<string>(), "MovingAI .map file")
        ("agents,a", po::value<string>(), "MovingAI .scen file")
        ("agentNum,k", po::value<int>()->default_value(0), "number of agents from .scen")
        ("output,o", po::value<string>(), "output file for paths")
        ("trace-json", po::value<string>(), "dump actual CBS search tree as JSON")
        ("no-schedule", po::bool_switch(),
         "suppress YAML schedule output (for benchmarks)")
        ("timeout,t", po::value<double>()->default_value(60), "timeout (seconds)")
        ("screen,s", po::value<int>()->default_value(1), "screen level (0/1/2)")
    ;

    po::options_description algorithm_options("Algorithm options");
    algorithm_options.add_options()
        ("mode", po::value<string>()->default_value("FOCBS"),
         "FOCBS=B-BZ-TB, FOCBS-noTB=B-BZ, FOCBS-noBZ=B-TB, FOCBS-H=H-BZ; also accepts H-BZ-TB and raw B/H modes")
        ("cost-mode", po::value<int>()->default_value(0), "oracle cost mode (0=SMU, Sum of Moves by Unassigned Agents; 1=optional NUA variant)")
        ("ten-horizon-mode", po::value<string>()->default_value("arrival"),
         "TEN horizon source: arrival=latest assigned-agent goal arrival; input=larger of .mapfua horizon and assigned-path makespan")
        ("static-idle", po::bool_switch(), "treat unassigned agents as static obstacles (baseline)")
        ("classic-return", po::bool_switch(), "convert unassigned agents to assigned agents with goal=start (baseline)")
        ("focal-lowlevel", po::bool_switch(), "use focal low-level search instead of the default LexAStar")
    ;

    po::options_description desc("FOCBS");
    desc.add(input_options).add(algorithm_options);
#endif

    po::options_description ablation_options("Independent ablation options");
    ablation_options.add_options()
        ("no-bz-ssp", po::bool_switch(), "disable BZ-SSP zero-cost warm-start solver")
        ("no-tiebreak", po::bool_switch(), "disable start-avoiding tie-breaking")
        ("no-early-term", po::bool_switch(), "run each SSP Dijkstra search until its queue is empty")
    ;

    desc.add(ablation_options);

    po::options_description conflict_options("CBS conflict handling");
    conflict_options.add_options()
        ("conflict-selection", po::value<string>()->default_value("earliest"),
         "choose ordinary conflicts: earliest or random (RNG seed 0)")
        ("conflict-detection", po::value<string>()->default_value("hash"),
         "full conflict detection: hash (vertex buckets and edge hash) or pairwise")
        ("incremental-conflicts", po::bool_switch(),
         "reuse unaffected conflicts when makespan is unchanged (default)")
        ("no-incremental-conflicts", po::bool_switch(),
         "recompute all conflicts at each CT node")
        ("random-conflict", po::bool_switch(),
         "alias for --conflict-selection random")
    ;
    desc.add(conflict_options);

    po::variables_map vm;
    int cli_style = po::command_line_style::default_style &
                    ~po::command_line_style::allow_guessing;
    try {
        po::store(po::command_line_parser(argc, argv)
                      .options(desc)
                      .style(cli_style)
                      .run(),
                  vm);
        if (vm.count("help")) { cout << desc << endl; return 0; }
        po::notify(vm);
    } catch (const po::error& e) {
        cerr << e.what() << endl;
        return 1;
    }

    const bool has_input = vm.count("input") != 0;
    const bool has_map = vm.count("map") != 0;
    const bool has_agents = vm.count("agents") != 0;
    if (has_input && (has_map || has_agents || !vm["agentNum"].defaulted())) {
        cerr << "Use either --input or --map/--agents/--agentNum, not both" << endl;
        return 1;
    }
    if (!has_input && (!has_map || !has_agents || vm["agentNum"].as<int>() <= 0)) {
        cerr << "Specify --input FILE, or --map FILE --agents FILE --agentNum N (N > 0)"
             << endl;
        return 1;
    }
    const double timeout = vm["timeout"].as<double>();
    if (!std::isfinite(timeout) || timeout <= 0) {
        cerr << "--timeout must be a finite positive number" << endl;
        return 1;
    }
    const int screen = vm["screen"].as<int>();
    if (screen < 0 || screen > 2) {
        cerr << "--screen must be 0, 1, or 2" << endl;
        return 1;
    }
#ifndef FOCBS_REVIEW_CLI
    if (vm["cost-mode"].as<int>() < 0 || vm["cost-mode"].as<int>() > 1) {
        cerr << "--cost-mode must be 0 or 1" << endl;
        return 1;
    }
#endif

    string requested_mode = vm["mode"].as<string>();
    ModeConfig mode_cfg;
#ifdef FOCBS_REVIEW_CLI
    if (!configureNamedMode(requested_mode, mode_cfg)) {
        cerr << "Invalid --mode: " << requested_mode
             << " (expected FOCBS, FOCBS-noTB, FOCBS-noBZ, FOCBS-H, "
                "B-BZ-TB, B-BZ, B-TB, H-BZ, or H-BZ-TB)" << endl;
        return 1;
    }
#else
    if (!configureNamedMode(requested_mode, mode_cfg)) {
        string raw_mode = requested_mode;
        if (raw_mode == "b") raw_mode = "B";
        else if (raw_mode == "h") raw_mode = "H";
        if (raw_mode != "B" && raw_mode != "H") {
            cerr << "Invalid --mode: " << requested_mode
                 << " (expected FOCBS, FOCBS-noTB, FOCBS-noBZ, FOCBS-H, "
                    "B-BZ-TB, B-BZ, B-TB, H-BZ, H-BZ-TB, B, or H)" << endl;
            return 1;
        }
        mode_cfg.internal_mode = raw_mode;
        mode_cfg.flow_conflict = raw_mode == "B";
    }
#endif

    // Named modes provide complete, reproducible defaults. Ablation
    // switches are applied afterwards so that a run can change one
    // component without silently changing branching or tie breaking.
    if (vm["no-bz-ssp"].as<bool>())
        mode_cfg.use_bz_ssp = false;
    if (vm["no-tiebreak"].as<bool>())
        mode_cfg.use_tiebreak = false;

    string mode = mode_cfg.internal_mode;
    bool flow_conflict = mode_cfg.flow_conflict;
    bool use_bz_ssp = mode_cfg.use_bz_ssp;
    bool use_tiebreak = mode_cfg.use_tiebreak;
    string conflict_selection = vm["conflict-selection"].as<string>();
    const string conflict_detection = vm["conflict-detection"].as<string>();
    if (conflict_selection != "earliest" && conflict_selection != "random") {
        cerr << "Invalid --conflict-selection: " << conflict_selection
             << " (expected earliest or random)" << endl;
        return 1;
    }
    if (conflict_detection != "hash" && conflict_detection != "pairwise") {
        cerr << "Invalid --conflict-detection: " << conflict_detection
             << " (expected hash or pairwise)" << endl;
        return 1;
    }
    if (vm["random-conflict"].as<bool>()) {
        if (!vm["conflict-selection"].defaulted() &&
            conflict_selection != "random") {
            cerr << "--random-conflict contradicts --conflict-selection earliest" << endl;
            return 1;
        }
        conflict_selection = "random";
    }
    const bool earliest_conflict = conflict_selection == "earliest";
    const bool incremental_conflicts = !vm["no-incremental-conflicts"].as<bool>();
    if (vm["incremental-conflicts"].as<bool>() && !incremental_conflicts) {
        cerr << "--incremental-conflicts contradicts --no-incremental-conflicts" << endl;
        return 1;
    }

#ifdef FOCBS_REVIEW_CLI
    string ten_horizon_mode = "arrival";
#else
    string ten_horizon_mode = vm["ten-horizon-mode"].as<string>();
#endif
    if (ten_horizon_mode != "arrival" && ten_horizon_mode != "input") {
        cerr << "Invalid --ten-horizon-mode: " << ten_horizon_mode
             << " (expected arrival or input)" << endl;
        return 1;
    }

    std::srand(0);

    Instance instance;
    try {
        if (has_input) {
            instance = Instance(vm["input"].as<string>());
        } else {
            instance = Instance(vm["map"].as<string>(), vm["agents"].as<string>(),
                                vm["agentNum"].as<int>());
        }
    } catch (const std::exception& e) {
        cerr << e.what() << endl;
        return 1;
    }

    // Classic-return converts each unassigned agent to an assigned agent with goal=start.
#ifndef FOCBS_REVIEW_CLI
    bool classic_return = vm["classic-return"].as<bool>();
    if (classic_return) {
        for (int loc : instance.idle_locations) {
            instance.start_locations.push_back(loc);
            instance.goal_locations.push_back(loc);
        }
        instance.num_of_agents += instance.num_of_idle;
        instance.idle_locations.clear();
        instance.num_of_idle = 0;
    }
#endif

    CBS cbs(instance, screen);
    if (ten_horizon_mode == "input")
        cbs.setTENHorizonMode(TENHorizonMode::Input);
    else
        cbs.setTENHorizonMode(TENHorizonMode::Arrival);
    cbs.setFlowConflict(flow_conflict);
    cbs.setExact(false);
#ifndef FOCBS_REVIEW_CLI
    cbs.setStaticIdle(vm["static-idle"].as<bool>());
#endif
    if (use_tiebreak) cbs.setTiebreak(true);
    cbs.setEarliestConflict(earliest_conflict);
    cbs.setConflictDetection(conflict_detection == "hash"
        ? ConflictDetection::Hash : ConflictDetection::Pairwise);
    cbs.setIncrementalConflicts(incremental_conflicts);
#ifndef FOCBS_REVIEW_CLI
    if (!vm["focal-lowlevel"].as<bool>())
        cbs.setLexAStar(true);
#else
    cbs.setLexAStar(true);
#endif
    if (use_bz_ssp) cbs.setBZSSP(true);
    if (vm["no-early-term"].as<bool>()) cbs.setNoEarlyTerm(true);
#ifndef FOCBS_REVIEW_CLI
    if (vm["cost-mode"].as<int>() != 0)
        cbs.setCostMode(static_cast<bzssp::CostMode>(vm["cost-mode"].as<int>()));
    if (vm.count("trace-json"))
        cbs.setTraceEnabled(true);
#endif

    cbs.solve(timeout);

#ifndef FOCBS_REVIEW_CLI
    if (vm.count("trace-json")) {
        string input_label;
        if (vm.count("input"))
            input_label = vm["input"].as<string>();
        else if (vm.count("map") && vm.count("agents"))
            input_label = vm["map"].as<string>() + " + " + vm["agents"].as<string>();
        if (!cbs.writeSearchTraceJson(vm["trace-json"].as<string>(), input_label)) {
            cerr << "Failed to write trace JSON: " << vm["trace-json"].as<string>() << endl;
            return 1;
        }
    }
#endif

    auto& oracle = cbs.getOracle();
    cout << std::setprecision(12);
    // FOCBS invokes the TEN oracle inside CBS, possibly many times.  Report
    // a wall-clock decomposition using the same steady-clock basis:
    // runtimeWall = runtimeCBS + runtimeTEN (up to floating-point rounding).
    const double runtime_ten = oracle.oracle_time;
    const double runtime_cbs = max(0.0, cbs.runtime_wall - runtime_ten);
    if (cbs.solution_found) {
        const auto& paths_ptr = cbs.getPaths();
        int sst = computeAssignedSST(instance, paths_ptr);
        int smu = computeIdleSMU(cbs.best_oracle.idle_paths);
        int fuel = computeAssignedFuel(paths_ptr) + smu;
        int makespan = 0;
        for (auto* p : paths_ptr)
            if (p) makespan = max(makespan, (int)p->size() - 1);
        for (const auto& ip : cbs.best_oracle.idle_paths)
            makespan = max(makespan, (int)ip.size() - 1);
        int nua = computeNUA(cbs.best_oracle.idle_paths);
        double avg_blockers = oracle.oracle_infeasible > 0
            ? oracle.total_blockers / (double)oracle.oracle_infeasible : 0.0;

        cout << "Planning successful!" << endl;
        cout << "statistics:" << endl;
        cout << "  timedOut: " << (cbs.timed_out ? 1 : 0) << endl;
        cout << "  exactProofComplete: " << (cbs.exact_proven ? 1 : 0) << endl;
        cout << "  mode: " << mode << endl;
        cout << "  flowConflictEnabled: " << (flow_conflict ? 1 : 0) << endl;
        cout << "  bzSSPEnabled: " << (use_bz_ssp ? 1 : 0) << endl;
        cout << "  tiebreakEnabled: " << (use_tiebreak ? 1 : 0) << endl;
        cout << "  tenHorizonMode: " << cbs.getTENHorizonModeName() << endl;
        cout << "  sst: " << sst << endl;
        cout << "  smu: " << smu << endl;
        cout << "  fuel: " << fuel << endl;
        cout << "  outputMode: computed" << endl;
        cout << "  scheduleMode: "
             << (vm["no-schedule"].as<bool>() ? "omitted" : "computed") << endl;
        cout << "  nua: " << nua << endl;
        int soc_all = sst + smu;
        cout << "  soc_all: " << soc_all << endl;
        cout << "  makespan: " << makespan << endl;
        cout << "  runtime: " << cbs.runtime << endl;
        cout << "  runtimeWall: " << cbs.runtime_wall << endl;
        cout << "  runtimeCBS: " << runtime_cbs << endl;
        cout << "  runtimeTEN: " << runtime_ten << endl;
        cout << "  runtimeTENBuild: " << oracle.runtime_ten_build << endl;
        cout << "  runtimeFlowSolve: " << oracle.runtime_flow_solve << endl;
        cout << "  runtimePathExtraction: " << oracle.runtime_path_extraction << endl;
        cout << "  runtimeDiagnosticTENBuild: " << oracle.runtime_diagnostic_ten_build << endl;
        cout << "  runtimeDiagnosticFlow: " << oracle.runtime_diagnostic_flow << endl;
        cout << "  runtimeBlockerExtraction: " << oracle.runtime_blocker_extraction << endl;
        cout << "  highLevelExpanded: " << cbs.num_HL_expanded << endl;
        cout << "  highLevelGenerated: " << cbs.num_HL_generated << endl;
        cout << "  lowLevelExpanded: " << cbs.num_LL_expanded << endl;
        cout << "  lowLevelGenerated: " << cbs.num_LL_generated << endl;
        cout << "  oracleCalls: " << oracle.oracle_calls << endl;
        cout << "  oracleInfeasible: " << oracle.oracle_infeasible << endl;
        cout << "  oracleTime: " << oracle.oracle_time << endl;
        cout << "  oracleFeasibleTime: " << oracle.oracle_feasible_time << endl;
        cout << "  oracleInfeasibleTime: " << oracle.oracle_infeasible_time << endl;
        cout << "  diagnosticCalls: " << oracle.diagnostic_calls << endl;
        cout << "  flowConflictBranches: " << cbs.flow_conflict_branches << endl;
        cout << "  totalBlockers: " << oracle.total_blockers << endl;
        cout << "  avgBlockers: " << avg_blockers << endl;
        cout << "  bzWarmStartTotal: " << oracle.bz_warmstart_total << endl;
        cout << "  bzZeroTotal: " << oracle.bz_zero_total << endl;
        printOracleInfeasibleNodeDistribution(cbs);
    } else {
        cout << "Planning NOT successful!" << endl;
        cout << "statistics:" << endl;
        cout << "  timedOut: " << (cbs.timed_out ? 1 : 0) << endl;
        cout << "  exactProofComplete: " << (cbs.exact_proven ? 1 : 0) << endl;
        cout << "  mode: " << mode << endl;
        cout << "  flowConflictEnabled: " << (flow_conflict ? 1 : 0) << endl;
        cout << "  bzSSPEnabled: " << (use_bz_ssp ? 1 : 0) << endl;
        cout << "  tiebreakEnabled: " << (use_tiebreak ? 1 : 0) << endl;
        cout << "  tenHorizonMode: " << cbs.getTENHorizonModeName() << endl;
        cout << "  outputMode: none" << endl;
        cout << "  scheduleMode: none" << endl;
        cout << "  runtime: " << cbs.runtime << endl;
        cout << "  runtimeWall: " << cbs.runtime_wall << endl;
        cout << "  runtimeCBS: " << runtime_cbs << endl;
        cout << "  runtimeTEN: " << runtime_ten << endl;
        cout << "  runtimeTENBuild: " << oracle.runtime_ten_build << endl;
        cout << "  runtimeFlowSolve: " << oracle.runtime_flow_solve << endl;
        cout << "  runtimePathExtraction: " << oracle.runtime_path_extraction << endl;
        cout << "  runtimeDiagnosticTENBuild: " << oracle.runtime_diagnostic_ten_build << endl;
        cout << "  runtimeDiagnosticFlow: " << oracle.runtime_diagnostic_flow << endl;
        cout << "  runtimeBlockerExtraction: " << oracle.runtime_blocker_extraction << endl;
        cout << "  highLevelExpanded: " << cbs.num_HL_expanded << endl;
        cout << "  highLevelGenerated: " << cbs.num_HL_generated << endl;
        cout << "  lowLevelExpanded: " << cbs.num_LL_expanded << endl;
        cout << "  lowLevelGenerated: " << cbs.num_LL_generated << endl;
        cout << "  oracleCalls: " << oracle.oracle_calls << endl;
        cout << "  oracleInfeasible: " << oracle.oracle_infeasible << endl;
        cout << "  oracleTime: " << oracle.oracle_time << endl;
        cout << "  oracleFeasibleTime: " << oracle.oracle_feasible_time << endl;
        cout << "  oracleInfeasibleTime: " << oracle.oracle_infeasible_time << endl;
        cout << "  diagnosticCalls: " << oracle.diagnostic_calls << endl;
        cout << "  flowConflictBranches: " << cbs.flow_conflict_branches << endl;
        cout << "  totalBlockers: " << oracle.total_blockers << endl;
        double avg_blockers = oracle.oracle_infeasible > 0
            ? oracle.total_blockers / (double)oracle.oracle_infeasible : 0.0;
        cout << "  avgBlockers: " << avg_blockers << endl;
        cout << "  bzWarmStartTotal: " << oracle.bz_warmstart_total << endl;
        cout << "  bzZeroTotal: " << oracle.bz_zero_total << endl;
        printOracleInfeasibleNodeDistribution(cbs);
    }

    cout << "  conflictSelection: " << conflict_selection << endl;
    cout << "  conflictDetection: " << conflict_detection << endl;
    cout << "  incrementalConflicts: " << (incremental_conflicts ? 1 : 0) << endl;
    cout << "  conflictScans: " << cbs.num_conflict_scans << endl;
    cout << "  conflictsDetected: " << cbs.num_conflicts_detected << endl;
    cout << "  runtimeConflictDetection: " << cbs.runtime_conflict_detection << endl;

    if (vm.count("output") && cbs.solution_found) {
        if (!cbs.savePaths(vm["output"].as<string>())) {
            cerr << "Failed to write paths: " << vm["output"].as<string>() << endl;
            return 1;
        }
    }

    // Print YAML-like schedules for downstream tools.
    if (cbs.solution_found && !vm["no-schedule"].as<bool>()) {
        int cols = instance.num_of_cols;
        cout << "assigned_schedule:" << endl;
        auto paths_ptr = cbs.getPaths();
        for (int i = 0; i < instance.num_of_agents; i++) {
            cout << "  agent" << i << ":" << endl;
            if (paths_ptr[i]) {
                for (int t = 0; t < (int)paths_ptr[i]->size(); t++) {
                    int loc = (*paths_ptr[i])[t].location;
                    cout << "    - x: " << loc % cols << endl
                         << "      y: " << loc / cols << endl
                         << "      t: " << t << endl;
                }
            }
        }
        cout << "unassigned_schedule:" << endl;
        for (size_t a = 0; a < cbs.best_oracle.idle_paths.size(); a++) {
            cout << "  unassigned" << a << ":" << endl;
            for (size_t t = 0; t < cbs.best_oracle.idle_paths[a].size(); t++) {
                cout << "    - x: " << cbs.best_oracle.idle_paths[a][t].first << endl
                     << "      y: " << cbs.best_oracle.idle_paths[a][t].second << endl
                     << "      t: " << t << endl;
            }
        }
    }

    cbs.clearSearchEngines();
    return 0;
}
