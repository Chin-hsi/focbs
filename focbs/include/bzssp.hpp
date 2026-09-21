#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bzssp {

using ll = long long;
static const ll INF = 1000000000000000000LL;

// CSR adjacency with paired forward and reverse arcs.

struct Graph {
    int N;                       // Number of nodes.
    int M;                       // Number of directed arcs, including reverse arcs.
    std::vector<int> head;       // head[arc] is the target node.
    std::vector<int> cost;       // cost[arc]
    std::vector<int> cap;        // Residual capacity.
    std::vector<int> adj_start;  // CSR start offsets.
    std::vector<int> adj_list;   // CSR arc list.

    void build_adjacency() {
        adj_start.assign(N + 1, 0);
        for (int i = 0; i < M; i++) {
            int u = head[i ^ 1];  // The paired reverse arc stores the tail.
            adj_start[u + 1]++;
        }
        for (int i = 1; i <= N; i++)
            adj_start[i] += adj_start[i - 1];
        adj_list.resize(M);
        std::vector<int> pos(adj_start.begin(), adj_start.end());
        for (int i = 0; i < M; i++) {
            int u = head[i ^ 1];
            adj_list[pos[u]++] = i;
        }
    }
};

// Binary min-heap with lazy deletion.

struct Heap {
    struct Entry { ll key; int node; };
    std::vector<Entry> data;
    int sz;

    void clear() { sz = 0; }
    bool empty() const { return sz == 0; }

    void push(ll key, int node) {
        if (sz >= (int)data.size()) data.push_back({key, node});
        else data[sz] = {key, node};
        int i = sz++;
        while (i > 0) {
            int p = (i - 1) >> 1;
            if (data[p].key <= data[i].key) break;
            std::swap(data[p], data[i]);
            i = p;
        }
    }

    Entry pop() {
        Entry top = data[0];
        data[0] = data[--sz];
        int i = 0;
        while (true) {
            int l = 2 * i + 1, r = 2 * i + 2, best = i;
            if (l < sz && data[l].key < data[best].key) best = l;
            if (r < sz && data[r].key < data[best].key) best = r;
            if (best == i) break;
            std::swap(data[i], data[best]);
            i = best;
        }
        return top;
    }
};

// Dial's bucket queue.

struct DialQueue {
    std::vector<std::vector<int>> buckets;
    int cur_d, max_d;

    void init(int) { buckets.resize(64); max_d = 0; cur_d = 0; }
    void clear() {
        for (int i = 0; i <= max_d && i < (int)buckets.size(); i++)
            buckets[i].clear();
        cur_d = 0; max_d = 0;
    }
    void push(int node, ll d) {
        int di = (int)d;
        if (di >= (int)buckets.size()) buckets.resize(di + 1);
        if (di > max_d) max_d = di;
        buckets[di].push_back(node);
    }
    bool empty() {
        while (cur_d <= max_d && buckets[cur_d].empty()) cur_d++;
        return cur_d > max_d;
    }
    std::pair<int, int> pop() {
        while (cur_d <= max_d && buckets[cur_d].empty()) cur_d++;
        int node = buckets[cur_d].back();
        buckets[cur_d].pop_back();
        return {cur_d, node};
    }
};

// SSP result.

struct SSPResult {
    ll total_cost;
    int augmentations;
    bool feasible;
    int bz_zero_count = 0;  // Zero-cost warm-start augmentations applied in Phase 1.
    struct AugInfo {
        int cost, path_len, reverse_used, settled_count;
        long long time_us;
    };
    std::vector<AugInfo> aug_info;
};

// SSP solver with Johnson potentials and Dijkstra.
// no_early_term disables stopping once the sink is settled.
// pq_mode: 0=binary heap, 1=Dial's bucket queue

inline SSPResult solve_ssp(Graph& g, int S, int T, int k,
                           bool no_early_term = false, int pq_mode = 0) {
    SSPResult res;
    res.total_cost = 0;
    res.augmentations = 0;
    res.feasible = true;

    int N = g.N;
    std::vector<ll> potential(N, 0);
    std::vector<ll> dist(N);
    std::vector<int> parent_arc(N);
    std::vector<int> settled;
    settled.reserve(N);

    Heap heap;
    DialQueue dial;
    if (pq_mode == 1) dial.init(N);

    for (int i = 0; i < k; i++) {
        auto t0 = std::chrono::high_resolution_clock::now();

        std::fill(dist.begin(), dist.end(), INF);
        std::fill(parent_arc.begin(), parent_arc.end(), -1);
        dist[S] = 0;
        settled.clear();

        if (pq_mode == 0) {
            heap.clear();
            heap.push(0, S);
            while (!heap.empty()) {
                Heap::Entry entry = heap.pop();
                ll d = entry.key; int u = entry.node;
                if (d > dist[u]) continue;
                settled.push_back(u);
                if (!no_early_term && u == T) break;
                for (int j = g.adj_start[u]; j < g.adj_start[u + 1]; j++) {
                    int aidx = g.adj_list[j];
                    if (g.cap[aidx] <= 0) continue;
                    int v = g.head[aidx];
                    ll rc = g.cost[aidx] + potential[u] - potential[v];
                    ll nd = d + rc;
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        parent_arc[v] = aidx;
                        heap.push(nd, v);
                    }
                }
            }
        } else {
            dial.clear();
            dial.push(S, 0);
            while (!dial.empty()) {
                std::pair<int,int> dp = dial.pop();
                int d = dp.first; int u = dp.second;
                if ((ll)d > dist[u]) continue;
                settled.push_back(u);
                if (!no_early_term && u == T) break;
                for (int j = g.adj_start[u]; j < g.adj_start[u + 1]; j++) {
                    int aidx = g.adj_list[j];
                    if (g.cap[aidx] <= 0) continue;
                    int v = g.head[aidx];
                    ll rc = g.cost[aidx] + potential[u] - potential[v];
                    ll nd = (ll)d + rc;
                    if (nd < dist[v]) {
                        dist[v] = nd;
                        parent_arc[v] = aidx;
                        dial.push(v, nd);
                    }
                }
            }
        }

        if (dist[T] >= INF) {
            res.feasible = false;
            break;
        }

        // Update Johnson potentials.
        if (no_early_term) {
            for (int vid = 0; vid < N; vid++)
                if (dist[vid] < INF) potential[vid] += dist[vid];
        } else {
            ll D = dist[T];
            for (int v : settled) potential[v] += dist[v] - D;
        }

        // Augment along the shortest path.
        int path_cost = 0, path_len = 0, reverse_used = 0;
        int v = T;
        while (v != S) {
            int aidx = parent_arc[v];
            int prev = g.head[aidx ^ 1];
            g.cap[aidx] -= 1;
            g.cap[aidx ^ 1] += 1;
            path_cost += g.cost[aidx];
            path_len++;
            if (aidx & 1) reverse_used++;
            v = prev;
        }

        res.total_cost += path_cost;
        res.augmentations++;

        auto t1 = std::chrono::high_resolution_clock::now();
        long long us = std::chrono::duration_cast<std::chrono::microseconds>(
            t1 - t0).count();
        res.aug_info.push_back({path_cost, path_len, reverse_used,
                                (int)settled.size(), us});
    }

    return res;
}

// BZ-SSP: batched zero-cost augmentation.
// Warm-start with zero-cost paths for unassigned-agent starts that remain unreserved.
// These paths reduce the number of later SSP augmentations.
// The warm start initializes partial flow; it does not freeze agent identity or trajectory.

inline int batch_zero_augment(Graph& g, int T,
                              const std::vector<int>& zero_wait_source_arcs) {
    int k0 = 0;
    // Eligibility and source-arc indices are recorded during TEN construction.
    // Preserve idle-start order without rescanning time layers or source arcs.
    for (int arc_s : zero_wait_source_arcs) {
        if (arc_s < 0 || g.cap[arc_s] <= 0) continue;

        // Trace the full path before updating residual capacity.
        // In SMU mode this is wait-in-place; in NUA mode it uses the private-wait chain.
        std::vector<int> path_arcs;
        path_arcs.push_back(arc_s);
        int cur = g.head[arc_s];

        while (cur != T) {
            int next_arc = -1;
            for (int j = g.adj_start[cur]; j < g.adj_start[cur + 1]; j++) {
                int aidx = g.adj_list[j];
                if ((aidx & 1) != 0 || g.cap[aidx] <= 0) continue;
                if (g.cost[aidx] != 0) continue;
                next_arc = aidx;
                break;
            }
            if (next_arc < 0) break;
            path_arcs.push_back(next_arc);
            cur = g.head[next_arc];
        }

        if (cur != T) continue;

        // Commit the zero-cost path into the residual network.
        // Later SSP steps can still reroute it through reverse arcs.
        for (int aidx : path_arcs) {
            g.cap[aidx] -= 1;
            g.cap[aidx ^ 1] += 1;
        }
        k0++;
    }
    return k0;
}

// BZ-SSP wrapper: Phase 1 warm start plus Phase 2 standard SSP.
inline SSPResult solve_ssp_bz(Graph& g, int S, int T, int k,
                              const std::vector<int>& zero_wait_source_arcs,
                              bool no_early_term = false, int pq_mode = 0) {
    int k0 = batch_zero_augment(g, T, zero_wait_source_arcs);
    int k_remaining = k - k0;
    SSPResult res;
    if (k_remaining > 0) {
        res = solve_ssp(g, S, T, k_remaining, no_early_term, pq_mode);
    } else {
        res.total_cost = 0;
        res.augmentations = 0;
        res.feasible = true;
    }
    // Merge warm-start and Phase 2 augmentation counts.
    res.augmentations += k0;
    res.bz_zero_count = k0;
    return res;
}

// TEN metadata.

struct TENInfo {
    Graph g;
    int S, T, k;
    int W, horizon;
    // node_type: 0=in, 1=out, 2=gadget_in, 3=gadget_out,
    //            4=super_s, 5=super_t, 6=private_wait (NUA)
    std::vector<int> node_cell;
    std::vector<int> node_time;
    std::vector<int> node_type;
    // In idle-start order: source arc if unreserved on [0,horizon], else -1.
    std::vector<int> zero_wait_source_arcs;
};

// Implicit TEN input.

struct ImplicitInput {
    int W, H_grid;
    int horizon;
    std::vector<bool> grid;     // grid[y*W+x] = passable
    int n_cells;
    std::vector<int> cell_list;
    std::vector<std::unordered_set<int>> resV;        // vertex reservations
    std::vector<std::unordered_set<long long>> resE;   // edge reservations
    std::vector<char> reserved_any;  // Cell reserved at any time in the horizon.
    int n_all_cells;
    std::vector<int> idle_starts;
    int n_idle;
    std::vector<std::vector<int>> neighbors;
};

// TEN cost modes.

enum class CostMode {
    SMU,   // Move arcs cost 1; total cost is Sum of Moves by Unassigned Agents.
    NUA    // Move arcs cost 0; each unassigned agent has activation cost 1.
           // Total cost is the number of unassigned agents that move.
};

// Build ImplicitInput from vectors.

inline ImplicitInput create_implicit_input(
        int W, int H_grid, int horizon,
        const std::vector<bool>& grid,
        const std::vector<std::vector<int>>& assigned_paths,
        const std::vector<int>& idle_starts) {
    ImplicitInput inp;
    inp.W = W;
    inp.H_grid = H_grid;
    inp.horizon = horizon;
    inp.n_all_cells = W * H_grid;
    inp.grid = grid;

    // Build the passable-cell list.
    inp.cell_list.clear();
    for (int cell = 0; cell < inp.n_all_cells; cell++) {
        if (cell < (int)inp.grid.size() && inp.grid[cell])
            inp.cell_list.push_back(cell);
    }
    inp.n_cells = (int)inp.cell_list.size();

    // Build assigned-agent reservations.
    inp.resV.resize(inp.horizon + 1);
    inp.resE.resize(inp.horizon);
    inp.reserved_any.assign(inp.n_all_cells, 0);
    for (const auto& path : assigned_paths) {
        if (path.empty()) continue;
        int plen = (int)path.size();
        for (int t = 0; t <= inp.horizon; t++) {
            int pos = (t < plen) ? path[t] : path[plen - 1];
            inp.resV[t].insert(pos);
            inp.reserved_any[pos] = 1;
        }
        for (int t = 0; t < inp.horizon; t++) {
            int u = (t < plen) ? path[t] : path[plen - 1];
            int v = (t + 1 < plen) ? path[t + 1] : path[plen - 1];
            if (u != v) {
                inp.resE[t].insert((long long)u * inp.n_all_cells + v);
            }
        }
    }

    inp.idle_starts = idle_starts;
    inp.n_idle = (int)inp.idle_starts.size();

    // Precompute grid neighbors.
    inp.neighbors.resize(inp.n_all_cells);
    int dx[] = {0, 0, 1, -1};
    int dy[] = {1, -1, 0, 0};
    for (int cell : inp.cell_list) {
        int y = cell / inp.W, x = cell % inp.W;
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx >= 0 && nx < inp.W && ny >= 0 && ny < inp.H_grid) {
                int nc = ny * inp.W + nx;
                if (inp.grid[nc])
                    inp.neighbors[cell].push_back(nc);
            }
        }
    }

    return inp;
}

// Implicit TEN construction.

inline TENInfo build_ten_implicit(ImplicitInput& inp,
                                  CostMode cost_mode = CostMode::SMU) {
    TENInfo ten;
    ten.W = inp.W;
    ten.horizon = inp.horizon;
    ten.k = inp.n_idle;
    int H = inp.horizon;
    int n_all = inp.n_all_cells;

    int flat_sz = n_all * (H + 1);
    std::vector<int> node_in(flat_sz, -1);
    std::vector<int> node_out(flat_sz, -1);

    int node_counter = 0;
    std::vector<int> tmp_cell, tmp_time, tmp_type;

    auto alloc_node = [&](int cell, int t, int type) -> int {
        int id = node_counter++;
        tmp_cell.push_back(cell);
        tmp_time.push_back(t);
        tmp_type.push_back(type);
        return id;
    };

    auto safe = [&](int cell, int t) -> bool {
        return inp.resV[t].find(cell) == inp.resV[t].end();
    };

    std::vector<int> arc_tails, arc_heads, arc_costs;
    const int move_cost = (cost_mode == CostMode::SMU) ? 1 : 0;

    // 1) Vertex-capacity arcs: in(v,t) -> out(v,t), cap=1, cost=0.
    for (int cell : inp.cell_list) {
        for (int t = 0; t <= H; t++) {
            if (!safe(cell, t)) continue;
            int idx = cell * (H + 1) + t;
            node_in[idx] = alloc_node(cell, t, 0);
            node_out[idx] = alloc_node(cell, t, 1);
            arc_tails.push_back(node_in[idx]);
            arc_heads.push_back(node_out[idx]);
            arc_costs.push_back(0);
        }
    }

    // 2) Wait arcs: out(v,t) -> in(v,t+1), cap=1, cost=0.
    for (int cell : inp.cell_list) {
        for (int t = 0; t < H; t++) {
            if (!safe(cell, t) || !safe(cell, t + 1)) continue;
            int out_id = node_out[cell * (H + 1) + t];
            int in_next = node_in[cell * (H + 1) + t + 1];
            if (out_id >= 0 && in_next >= 0) {
                arc_tails.push_back(out_id);
                arc_heads.push_back(in_next);
                arc_costs.push_back(0);
            }
        }
    }

    // 3) Move arcs plus anti-swap gadgets.
    struct UEdge { int a, b; };
    std::vector<UEdge> edges;
    for (int cell : inp.cell_list) {
        for (int nb : inp.neighbors[cell]) {
            if (cell < nb)
                edges.push_back({cell, nb});
        }
    }

    for (int t = 0; t < H; t++) {
        for (auto& e : edges) {
            int a = e.a, b = e.b;
            bool allow_ab = safe(a, t) && safe(b, t + 1) &&
                inp.resE[t].find((long long)b * n_all + a) == inp.resE[t].end();
            bool allow_ba = safe(b, t) && safe(a, t + 1) &&
                inp.resE[t].find((long long)a * n_all + b) == inp.resE[t].end();
            if (!allow_ab && !allow_ba) continue;

            int g_in = alloc_node(a, t, 2);
            int g_out = alloc_node(a, t, 3);
            arc_tails.push_back(g_in);
            arc_heads.push_back(g_out);
            arc_costs.push_back(0);

            if (allow_ab) {
                int a_out = node_out[a * (H + 1) + t];
                int b_in_next = node_in[b * (H + 1) + t + 1];
                if (a_out >= 0 && b_in_next >= 0) {
                    arc_tails.push_back(a_out);
                    arc_heads.push_back(g_in);
                    arc_costs.push_back(move_cost);
                    arc_tails.push_back(g_out);
                    arc_heads.push_back(b_in_next);
                    arc_costs.push_back(0);
                }
            }
            if (allow_ba) {
                int b_out = node_out[b * (H + 1) + t];
                int a_in_next = node_in[a * (H + 1) + t + 1];
                if (b_out >= 0 && a_in_next >= 0) {
                    arc_tails.push_back(b_out);
                    arc_heads.push_back(g_in);
                    arc_costs.push_back(move_cost);
                    arc_tails.push_back(g_out);
                    arc_heads.push_back(a_in_next);
                    arc_costs.push_back(0);
                }
            }
        }
    }

    // 4) Super source / sink
    int S = alloc_node(-1, -1, 4);
    int T = alloc_node(-1, -1, 5);

    ten.zero_wait_source_arcs.assign(inp.n_idle, -1);
    for (int i = 0; i < inp.n_idle; i++) {
        int cell = inp.idle_starts[i];
        // Paired forward arcs retain index 2 * raw_index after adjacency build.
        if (!inp.reserved_any[cell])
            ten.zero_wait_source_arcs[i] = 2 * (int)arc_tails.size();

        if (cost_mode == CostMode::SMU) {
            // SMU mode: S connects directly to in(start, 0).
            int in_id = node_in[cell * (H + 1) + 0];
            if (in_id < 0) {
                std::fprintf(stderr, "Idle start cell %d blocked at t=0\n", cell);
                ten.g.N = 0;
                return ten;
            }
            arc_tails.push_back(S);
            arc_heads.push_back(in_id);
            arc_costs.push_back(0);
        } else {
            // NUA mode: private wait chain plus activation edge.
            // pw(t) represents an unassigned agent waiting at its start cell.
            std::vector<int> pw(H + 1);
            for (int t = 0; t <= H; t++)
                pw[t] = alloc_node(cell, t, 6);

            // S -> pw(0).
            arc_tails.push_back(S);
            arc_heads.push_back(pw[0]);
            arc_costs.push_back(0);

            // pw(t) -> pw(t+1): wait if the cell is safe at both layers.
            for (int t = 0; t < H; t++) {
                if (!safe(cell, t) || !safe(cell, t + 1)) continue;
                arc_tails.push_back(pw[t]);
                arc_heads.push_back(pw[t + 1]);
                arc_costs.push_back(0);
            }

            // pw(H) -> T: finish without moving if the final layer is safe.
            if (safe(cell, H)) {
                arc_tails.push_back(pw[H]);
                arc_heads.push_back(T);
                arc_costs.push_back(0);
            }

            // pw(t) -> in(cell, t): activation edge with cost 1.
            for (int t = 0; t <= H; t++) {
                int in_id = node_in[cell * (H + 1) + t];
                if (in_id < 0) continue;
                arc_tails.push_back(pw[t]);
                arc_heads.push_back(in_id);
                arc_costs.push_back(1);
            }
        }
    }

    for (int cell : inp.cell_list) {
        if (!safe(cell, H)) continue;
        int out_id = node_out[cell * (H + 1) + H];
        if (out_id >= 0) {
            arc_tails.push_back(out_id);
            arc_heads.push_back(T);
            arc_costs.push_back(0);
        }
    }

    // Build paired-arc adjacency.
    int n_raw = (int)arc_tails.size();
    int n_nodes = node_counter;

    ten.g.N = n_nodes;
    ten.g.M = n_raw * 2;
    ten.g.head.resize(ten.g.M);
    ten.g.cost.resize(ten.g.M);
    ten.g.cap.resize(ten.g.M);

    for (int i = 0; i < n_raw; i++) {
        int fwd = 2 * i;
        int rev = 2 * i + 1;
        ten.g.head[fwd] = arc_heads[i];
        ten.g.cost[fwd] = arc_costs[i];
        ten.g.cap[fwd] = 1;
        ten.g.head[rev] = arc_tails[i];
        ten.g.cost[rev] = -arc_costs[i];
        ten.g.cap[rev] = 0;
    }

    ten.S = S;
    ten.T = T;
    ten.node_cell = std::move(tmp_cell);
    ten.node_time = std::move(tmp_time);
    ten.node_type = std::move(tmp_type);

    ten.g.build_adjacency();
    return ten;
}

// Flow path extraction result.

struct FlowPaths {
    int k;
    int horizon;
    // paths[i] is unassigned agent i's (x, y) coordinate at each timestep.
    std::vector<std::vector<std::pair<int, int>>> paths;
};

// Extract unassigned-agent paths from the solved TEN.
// Track consumed arcs without mutating residual capacities.

inline FlowPaths extract_paths(TENInfo& ten) {
    FlowPaths out;
    out.k = ten.k;
    out.horizon = ten.horizon;
    out.paths.reserve(out.k);

    Graph& g = ten.g;
    int S = ten.S, T = ten.T;
    int H = ten.horizon;
    int W = ten.W;
    std::vector<char> used(g.M, 0);

    // Find an unused forward arc carrying flow from node u.
    auto take_flow_arc = [&](int u) -> int {
        for (int j = g.adj_start[u]; j < g.adj_start[u + 1]; j++) {
            int aidx = g.adj_list[j];
            // Forward arc, saturated by flow, and not traced yet.
            if ((aidx & 1) == 0 && g.cap[aidx] == 0 && !used[aidx]) {
                used[aidx] = 1;
                return aidx;
            }
        }
        return -1;
    };

    for (int idx = 0; idx < ten.k; idx++) {
        int aidx = take_flow_arc(S);
        if (aidx < 0) {
            std::fprintf(stderr, "Failed to find flow for unassigned agent %d\n", idx);
            out.paths.clear();
            return out;
        }

        int cur = g.head[aidx];
        // Record cells by timestep, including NUA private-wait nodes.
        std::vector<int> path_cells(H + 1, -1);

        while (cur != T) {
            int type = ten.node_type[cur];
            // type 0 (in) and type 6 (private_wait) both encode a cell at a timestep.
            if (type == 0 || type == 6) {
                int t = ten.node_time[cur];
                if (t >= 0 && t <= H) {
                    path_cells[t] = ten.node_cell[cur];
                }
            }

            aidx = take_flow_arc(cur);
            if (aidx < 0) {
                std::fprintf(stderr,
                    "Path trace stuck at node %d (cell=%d, t=%d, type=%d)\n",
                    cur, ten.node_cell[cur], ten.node_time[cur], type);
                out.paths.clear();
                return out;
            }
            cur = g.head[aidx];
        }

        if (path_cells[0] < 0) {
            std::fprintf(stderr, "Extracted empty path for unassigned agent %d\n", idx);
            out.paths.clear();
            return out;
        }

        // Fill missing timesteps when the path is on out/gadget nodes.
        for (int t = 1; t <= H; t++) {
            if (path_cells[t] < 0) path_cells[t] = path_cells[t - 1];
        }

        // Convert cell IDs to (x, y) coordinates.
        std::vector<std::pair<int, int>> coords;
        coords.reserve(H + 1);
        for (int t = 0; t <= H; t++) {
            coords.push_back({path_cells[t] % W, path_cells[t] / W});
        }
        out.paths.push_back(std::move(coords));
    }

    return out;
}

// Diagnostic TEN for flow-conflict extraction.
// Build this diagnostic TEN when the strict oracle is infeasible.
// The resulting min-cut blockers drive FOCBS flow-conflict branching.
//
// Differences from the strict TEN:
//   - Assigned-occupied (cell,t) nodes remain, but their capacity arcs have cap=0.
//   - Assigned-blocked swap arcs also remain with cap=0.
//   - These zero-capacity arcs carry owner metadata for blocker extraction.
//
// Correctness sketch:
//   Fixed assigned paths and horizon H are feasible iff maxflow(N_tau^H)=|U|.
//   If maxflow < |U|, the source-reachable cut contains locked arcs B.
//   At the same horizon H, retaining all reservations in B remains infeasible.
//   Full flow at this horizon requires releasing at least one reservation in B.

enum class ConflictArcKind {
    Normal,
    LockedVertex,   // Vertex-capacity arc occupied by an assigned agent.
    LockedEdge      // Edge arc blocked by an assigned-agent swap.
};

struct ConflictArcMeta {
    ConflictArcKind kind = ConflictArcKind::Normal;
    int owner = -1;    // Assigned agent that owns this reservation.
    int time = -1;     // Timestep.
    int cell = -1;     // Cell for a vertex blocker.
    int from = -1;     // Edge blocker: assigned move from -> to.
    int to = -1;
};

struct ConflictTENInfo {
    Graph g;
    int S, T, k;
    int W, horizon;
    std::vector<int> node_cell;
    std::vector<int> node_time;
    std::vector<int> node_type;
    std::vector<ConflictArcMeta> arc_meta;  // Same size as g.M.
};

struct FlowBlocker {
    enum class Type { Vertex, Edge };
    Type type = Type::Vertex;
    int owner = -1;
    int time = -1;
    int cell = -1;
    int from = -1;
    int to = -1;

    bool operator<(const FlowBlocker& o) const {
        return std::tie(type, owner, time, cell, from, to) <
               std::tie(o.type, o.owner, o.time, o.cell, o.from, o.to);
    }
    bool operator==(const FlowBlocker& o) const {
        return std::tie(type, owner, time, cell, from, to) ==
               std::tie(o.type, o.owner, o.time, o.cell, o.from, o.to);
    }
};

struct FlowConflict {
    int flow_value = 0;
    int deficit = 0;    // k - flow_value
    std::vector<FlowBlocker> blockers;
};

// Build conflict-TEN input.

struct ConflictImplicitInput {
    int W, H_grid, horizon;
    int n_all_cells, n_cells;
    std::vector<bool> grid;
    std::vector<int> cell_list;
    std::vector<int> idle_starts;
    int n_idle;
    std::vector<std::vector<int>> neighbors;

    // t -> cell -> assigned owner.
    std::vector<std::unordered_map<int, int>> resV_owner;
    // t -> (u * n_all_cells + v) -> assigned owner for directed move u -> v.
    std::vector<std::unordered_map<ll, int>> resE_owner;
};

inline ConflictImplicitInput create_conflict_implicit_input(
        int W, int H_grid, int horizon,
        const std::vector<bool>& grid,
        const std::vector<std::vector<int>>& assigned_paths,
        const std::vector<int>& idle_starts) {
    ConflictImplicitInput inp;
    inp.W = W;
    inp.H_grid = H_grid;
    inp.horizon = horizon;
    inp.n_all_cells = W * H_grid;
    inp.grid = grid;
    inp.idle_starts = idle_starts;
    inp.n_idle = (int)idle_starts.size();

    inp.cell_list.clear();
    for (int cell = 0; cell < inp.n_all_cells; ++cell) {
        if (cell < (int)grid.size() && grid[cell])
            inp.cell_list.push_back(cell);
    }
    inp.n_cells = (int)inp.cell_list.size();

    inp.neighbors.resize(inp.n_all_cells);
    int dx[] = {0, 0, 1, -1};
    int dy[] = {1, -1, 0, 0};
    for (int cell : inp.cell_list) {
        int y = cell / W, x = cell % W;
        for (int d = 0; d < 4; ++d) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H_grid) continue;
            int nc = ny * W + nx;
            if (grid[nc]) inp.neighbors[cell].push_back(nc);
        }
    }

    inp.resV_owner.resize(horizon + 1);
    inp.resE_owner.resize(horizon);
    for (size_t agent = 0; agent < assigned_paths.size(); ++agent) {
        const auto& path = assigned_paths[agent];
        if (path.empty()) continue;
        int plen = (int)path.size();
        for (int t = 0; t <= horizon; ++t) {
            int cell = (t < plen) ? path[t] : path[plen - 1];
            inp.resV_owner[t][cell] = (int)agent;
        }
        for (int t = 0; t < horizon; ++t) {
            int u = (t < plen) ? path[t] : path[plen - 1];
            int v = (t + 1 < plen) ? path[t + 1] : path[plen - 1];
            if (u != v)
                inp.resE_owner[t][(ll)u * inp.n_all_cells + v] = (int)agent;
        }
    }
    return inp;
}

// Conflict TEN construction.
// Similar to build_ten_implicit, but reserved cell-times remain as explicit cap=0 arcs.

inline ConflictTENInfo build_ten_conflict_implicit(
        const ConflictImplicitInput& inp) {
    ConflictTENInfo ten;
    ten.W = inp.W;
    ten.horizon = inp.horizon;
    ten.k = inp.n_idle;
    int H = inp.horizon;
    int n_all = inp.n_all_cells;

    int flat_sz = n_all * (H + 1);
    std::vector<int> node_in(flat_sz, -1);
    std::vector<int> node_out(flat_sz, -1);

    int node_counter = 0;
    std::vector<int> tmp_cell, tmp_time, tmp_type;

    auto alloc_node = [&](int cell, int t, int type) -> int {
        int id = node_counter++;
        tmp_cell.push_back(cell);
        tmp_time.push_back(t);
        tmp_type.push_back(type);
        return id;
    };

    struct RawArc {
        int tail, head, cap, cost;
        ConflictArcMeta meta;
    };
    std::vector<RawArc> raw;

    auto add_arc = [&](int tail, int head, int cap, int cost,
                       const ConflictArcMeta& meta = ConflictArcMeta()) {
        raw.push_back({tail, head, cap, cost, meta});
    };

    auto v_owner = [&](int cell, int t) -> int {
        const auto& m = inp.resV_owner[t];
        auto it = m.find(cell);
        return it == m.end() ? -1 : it->second;
    };
    auto e_owner = [&](int from, int to, int t) -> int {
        const auto& m = inp.resE_owner[t];
        auto it = m.find((ll)from * n_all + to);
        return it == m.end() ? -1 : it->second;
    };

    // 1) Allocate in/out nodes for every passable cell-time.
    for (int cell : inp.cell_list) {
        for (int t = 0; t <= H; ++t) {
            int idx = cell * (H + 1) + t;
            node_in[idx] = alloc_node(cell, t, 0);
            node_out[idx] = alloc_node(cell, t, 1);
        }
    }

    // 2) Vertex-capacity arcs: in(v,t) -> out(v,t).
    //    Reserved: cap=0 plus LockedVertex metadata.
    //    Free: cap=1.
    for (int cell : inp.cell_list) {
        for (int t = 0; t <= H; ++t) {
            int idx = cell * (H + 1) + t;
            int owner = v_owner(cell, t);
            if (owner >= 0) {
                ConflictArcMeta m;
                m.kind = ConflictArcKind::LockedVertex;
                m.owner = owner;
                m.time = t;
                m.cell = cell;
                add_arc(node_in[idx], node_out[idx], 0, 0, m);
            } else {
                add_arc(node_in[idx], node_out[idx], 1, 0);
            }
        }
    }

    // 3) Wait arcs: out(v,t) -> in(v,t+1), cap=1, cost=0.
    for (int cell : inp.cell_list) {
        for (int t = 0; t < H; ++t) {
            int out_id = node_out[cell * (H + 1) + t];
            int in_next = node_in[cell * (H + 1) + t + 1];
            if (out_id >= 0 && in_next >= 0) {
                add_arc(out_id, in_next, 1, 0);
            }
        }
    }

    // 4) Move arcs plus anti-swap gadgets.
    struct UEdge { int a, b; };
    std::vector<UEdge> edges;
    for (int cell : inp.cell_list) {
        for (int nb : inp.neighbors[cell]) {
            if (cell < nb)
                edges.push_back({cell, nb});
        }
    }

    for (int t = 0; t < H; ++t) {
        for (const auto& e : edges) {
            int a = e.a, b = e.b;
            // Gadget nodes enforce shared capacity for opposite moves.
            int g_in = alloc_node(a, t, 2);
            int g_out = alloc_node(a, t, 3);
            add_arc(g_in, g_out, 1, 0);  // gadget capacity

            // Unassigned move a -> b: entry arc out(a,t) -> g_in.
            // If assigned move b -> a blocks it, cap=0 plus LockedEdge metadata.
            int owner_ba = e_owner(b, a, t);
            if (owner_ba >= 0) {
                ConflictArcMeta m;
                m.kind = ConflictArcKind::LockedEdge;
                m.owner = owner_ba;
                m.time = t;
                m.from = b;
                m.to = a;
                add_arc(node_out[a * (H + 1) + t], g_in, 0, 1, m);
            } else {
                add_arc(node_out[a * (H + 1) + t], g_in, 1, 1);
            }
            // Exit arc g_out -> in(b,t+1).
            add_arc(g_out, node_in[b * (H + 1) + t + 1], 1, 0);

            // Unassigned move b -> a: entry arc out(b,t) -> g_in.
            int owner_ab = e_owner(a, b, t);
            if (owner_ab >= 0) {
                ConflictArcMeta m;
                m.kind = ConflictArcKind::LockedEdge;
                m.owner = owner_ab;
                m.time = t;
                m.from = a;
                m.to = b;
                add_arc(node_out[b * (H + 1) + t], g_in, 0, 1, m);
            } else {
                add_arc(node_out[b * (H + 1) + t], g_in, 1, 1);
            }
            // exit arc g_out → in(a,t+1)
            add_arc(g_out, node_in[a * (H + 1) + t + 1], 1, 0);
        }
    }

    // 5) Super source / sink
    int S = alloc_node(-1, -1, 4);
    int T = alloc_node(-1, -1, 5);

    for (int cell : inp.idle_starts) {
        int in_id = node_in[cell * (H + 1) + 0];
        if (in_id < 0) {
            ten.g.N = 0;
            return ten;
        }
        add_arc(S, in_id, 1, 0);
    }

    for (int cell : inp.cell_list) {
        int out_id = node_out[cell * (H + 1) + H];
        if (out_id >= 0) {
            add_arc(out_id, T, 1, 0);
        }
    }

    // Build paired-arc adjacency.
    int n_raw = (int)raw.size();
    ten.g.N = node_counter;
    ten.g.M = n_raw * 2;
    ten.g.head.resize(ten.g.M);
    ten.g.cost.resize(ten.g.M);
    ten.g.cap.resize(ten.g.M);
    ten.arc_meta.assign(ten.g.M, ConflictArcMeta());

    for (int i = 0; i < n_raw; ++i) {
        int fwd = 2 * i;
        int rev = fwd + 1;
        ten.g.head[fwd] = raw[i].head;
        ten.g.cost[fwd] = raw[i].cost;
        ten.g.cap[fwd] = raw[i].cap;
        ten.g.head[rev] = raw[i].tail;
        ten.g.cost[rev] = -raw[i].cost;
        ten.g.cap[rev] = 0;
        ten.arc_meta[fwd] = raw[i].meta;
    }

    ten.S = S;
    ten.T = T;
    ten.node_cell = std::move(tmp_cell);
    ten.node_time = std::move(tmp_time);
    ten.node_type = std::move(tmp_type);
    ten.g.build_adjacency();
    return ten;
}

// Extract flow conflicts from the failed SSP residual graph.
// Preconditions: solve_ssp has run on ten.g and ssp.augmentations < ten.k.
//
// Algorithm:
//   1. BFS from S over residual arcs to find the source-reachable set R.
//   2. Each locked forward arc crossing from R to V\R is a blocker.
//   3. Each blocker maps to an assigned reservation whose release may restore feasibility.

inline FlowConflict extract_flow_conflict(
        const ConflictTENInfo& ten,
        int achieved_flow) {
    FlowConflict out;
    out.flow_value = achieved_flow;
    out.deficit = ten.k - achieved_flow;

    const Graph& g = ten.g;
    std::vector<char> vis(g.N, 0);
    std::vector<int> queue;
    queue.reserve(g.N);
    vis[ten.S] = 1;
    queue.push_back(ten.S);

    for (size_t qi = 0; qi < queue.size(); ++qi) {
        int u = queue[qi];
        for (int j = g.adj_start[u]; j < g.adj_start[u + 1]; ++j) {
            int aidx = g.adj_list[j];
            if (g.cap[aidx] <= 0) continue;
            int v = g.head[aidx];
            if (!vis[v]) {
                vis[v] = 1;
                queue.push_back(v);
            }
        }
    }

    // Collect locked arcs crossing the cut.
    for (int u = 0; u < g.N; ++u) {
        if (!vis[u]) continue;
        for (int j = g.adj_start[u]; j < g.adj_start[u + 1]; ++j) {
            int aidx = g.adj_list[j];
            if (aidx & 1) continue;  // Inspect forward arcs only.
            int v = g.head[aidx];
            if (vis[v]) continue;  // The arc does not cross the cut.

            const ConflictArcMeta& meta = ten.arc_meta[aidx];
            if (meta.kind == ConflictArcKind::LockedVertex) {
                FlowBlocker b;
                b.type = FlowBlocker::Type::Vertex;
                b.owner = meta.owner;
                b.time = meta.time;
                b.cell = meta.cell;
                out.blockers.push_back(b);
            } else if (meta.kind == ConflictArcKind::LockedEdge) {
                FlowBlocker b;
                b.type = FlowBlocker::Type::Edge;
                b.owner = meta.owner;
                b.time = meta.time;
                b.from = meta.from;
                b.to = meta.to;
                out.blockers.push_back(b);
            }
        }
    }

    // Deduplicate and sort.
    std::sort(out.blockers.begin(), out.blockers.end());
    out.blockers.erase(
        std::unique(out.blockers.begin(), out.blockers.end()),
        out.blockers.end());
    return out;
}

}  // namespace bzssp
