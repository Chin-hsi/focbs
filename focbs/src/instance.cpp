#include "instance.h"
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace {
[[noreturn]] void inputError(const string& fname, const string& message) {
    throw std::runtime_error(fname + ": " + message);
}

string getParentDir(const string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos == string::npos)
        return "";
    return path.substr(0, pos + 1);
}

bool isAbsolutePath(const string& path) {
    if (path.empty())
        return false;
    if (path[0] == '/' || path[0] == '\\')
        return true;
    return path.size() > 1 && path[1] == ':';
}

string resolveSiblingPath(const string& inst_path, const string& leaf) {
    if (isAbsolutePath(leaf))
        return leaf;
    return getParentDir(inst_path) + leaf;
}
}

Instance::Instance(const string& mapfua_fname) {
    loadMapfua(mapfua_fname);
}

Instance::Instance(const string& map_fname, const string& scen_fname, int num_agents)
    : num_of_idle(0), horizon(0) {
    loadMovingAIMap(map_fname);
    loadMovingAIScen(scen_fname, num_agents);
}

bool Instance::validMove(int curr, int next) const {
    if (next < 0 || next >= map_size) return false;
    if (my_map[next]) return false;
    return getManhattanDistance(curr, next) < 2;
}

list<int> Instance::getNeighbors(int curr) const {
    list<int> neighbors;
    int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
    for (int next : candidates)
        if (validMove(curr, next))
            neighbors.emplace_back(next);
    return neighbors;
}

bool Instance::loadMapfua(const string& fname) {
    std::ifstream f(fname);
    if (!f) inputError(fname, "cannot open input file");

    string magic;
    if (!(f >> magic) || (magic != "mapfua" && magic != "mapfua_v2")) {
        inputError(fname, "expected mapfua or mapfua_v2 header");
    }

    string key;
    string map_name;
    if (!(f >> key >> map_name) || key != "map") {
        inputError(fname, "missing map line");
    }
    loadMovingAIMap(resolveSiblingPath(fname, map_name));

    if (!(f >> key >> horizon) || key != "horizon" || horizon < 0) {
        inputError(fname, "invalid horizon");
    }

    if (!(f >> key >> num_of_agents) || key != "assigned" || num_of_agents < 0) {
        inputError(fname, "invalid assigned section");
    }
    const int free_cells = static_cast<int>(std::count(my_map.begin(), my_map.end(), false));
    if (num_of_agents > free_cells)
        inputError(fname, "assigned count exceeds the number of free cells");
    start_locations.resize(num_of_agents);
    goal_locations.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++) {
        int sx, sy, gx, gy;
        if (!(f >> sx >> sy >> gx >> gy)) {
            inputError(fname, "malformed assigned agent line " + std::to_string(i + 1));
        }
        start_locations[i] = checkedLocation(sx, sy, fname);
        goal_locations[i] = checkedLocation(gx, gy, fname);
    }

    if (!(f >> key >> num_of_idle) || key != "unassigned" || num_of_idle < 0) {
        inputError(fname, "invalid unassigned section");
    }
    if (num_of_idle > free_cells - num_of_agents)
        inputError(fname, "total agent count exceeds the number of free cells");
    idle_locations.resize(num_of_idle);
    for (int i = 0; i < num_of_idle; i++) {
        int x, y;
        if (!(f >> x >> y)) {
            inputError(fname, "malformed unassigned agent line " + std::to_string(i + 1));
        }
        idle_locations[i] = checkedLocation(x, y, fname);
    }
    validateLocations(fname);
    return true;
}

bool Instance::loadMovingAIMap(const string& fname) {
    std::ifstream f(fname);
    if (!f) inputError(fname, "cannot open map file");

    string line;
    string key, value, extra;
    std::getline(f, line);
    std::istringstream type_line(line);
    if (!(type_line >> key >> value) || key != "type" || value != "octile" ||
        (type_line >> extra))
        inputError(fname, "expected type octile header");

    auto read_dimension = [&](const string& expected, int& dimension) {
        if (!std::getline(f, line)) inputError(fname, "missing " + expected);
        std::istringstream header(line);
        if (!(header >> key >> dimension) || key != expected || dimension <= 0 ||
            (header >> extra))
            inputError(fname, "invalid " + expected);
    };
    read_dimension("height", num_of_rows);
    read_dimension("width", num_of_cols);
    if (static_cast<long long>(num_of_rows) * num_of_cols > INT_MAX)
        inputError(fname, "map dimensions exceed the supported range");
    std::getline(f, line);
    std::istringstream map_line(line);
    if (!(map_line >> key) || key != "map" || (map_line >> extra))
        inputError(fname, "missing map grid header");

    map_size = num_of_rows * num_of_cols;
    my_map.assign(map_size, false);
    for (int r = 0; r < num_of_rows; r++) {
        if (!std::getline(f, line)) inputError(fname, "missing map row " + std::to_string(r + 1));
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.size() != static_cast<size_t>(num_of_cols))
            inputError(fname, "incorrect width in map row " + std::to_string(r + 1));
        for (int c = 0; c < num_of_cols; c++) {
            const char terrain = line[c];
            if (string(".GS@OTW").find(terrain) == string::npos)
                inputError(fname, "unsupported map terrain character");
            my_map[r * num_of_cols + c] =
                terrain != '.' && terrain != 'G' && terrain != 'S';
        }
    }
    return true;
}

bool Instance::loadMovingAIScen(const string& fname, int num_agents) {
    std::ifstream f(fname);
    if (!f) inputError(fname, "cannot open scenario file");

    string line;
    std::getline(f, line);
    std::istringstream header(line);
    string key, extra;
    double version;
    if (!(header >> key >> version) || key != "version" || version != 1 ||
        (header >> extra))
        inputError(fname, "expected version 1 header");
    const int free_cells = static_cast<int>(std::count(my_map.begin(), my_map.end(), false));
    if (num_agents <= 0 || num_agents > free_cells)
        inputError(fname, "agent count must be positive and no larger than the number of free cells");

    num_of_agents = num_agents;
    num_of_idle = 0;
    horizon = 0;
    start_locations.resize(num_of_agents);
    goal_locations.resize(num_of_agents);

    for (int i = 0; i < num_of_agents; i++) {
        if (!std::getline(f, line))
            inputError(fname, "fewer scenario rows than the requested agent count");
        std::istringstream row(line);
        int bucket, cols, rows, sc, sr, gc, gr;
        string map_name;
        double distance;
        if (!(row >> bucket >> map_name >> cols >> rows >> sc >> sr >> gc >> gr >> distance) ||
            bucket < 0 || !std::isfinite(distance) || distance < 0 || (row >> extra))
            inputError(fname, "malformed scenario row " + std::to_string(i + 1));
        if (cols != num_of_cols || rows != num_of_rows)
            inputError(fname, "scenario dimensions do not match the map");
        start_locations[i] = checkedLocation(sc, sr, fname);
        goal_locations[i] = checkedLocation(gc, gr, fname);
    }
    validateLocations(fname);
    return true;
}

int Instance::checkedLocation(int x, int y, const string& fname) const {
    if (x < 0 || x >= num_of_cols || y < 0 || y >= num_of_rows)
        inputError(fname, "coordinate outside the map: " + std::to_string(x) + "," + std::to_string(y));
    const int loc = linearizeCoordinate(y, x); // x is the column; y is the row.
    if (my_map[loc])
        inputError(fname, "coordinate lies on an obstacle: " + std::to_string(x) + "," + std::to_string(y));
    return loc;
}

void Instance::validateLocations(const string& fname) const {
    std::unordered_set<int> starts, goals;
    for (int loc : start_locations)
        if (!starts.insert(loc).second) inputError(fname, "duplicate start location");
    for (int loc : idle_locations)
        if (!starts.insert(loc).second) inputError(fname, "duplicate start location");
    for (int loc : goal_locations)
        if (!goals.insert(loc).second) inputError(fname, "duplicate target location");
}
