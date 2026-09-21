#pragma once
#include "common.h"

// Supports .mapfua and MovingAI .map + .scen inputs.
class Instance {
public:
    int num_of_cols = 0;
    int num_of_rows = 0;
    int map_size = 0;
    int num_of_agents = 0;  // assigned agents only
    int num_of_idle = 0;
    int horizon = 0;

    vector<int> start_locations;   // assigned-agent starts
    vector<int> goal_locations;    // assigned-agent targets
    vector<int> idle_locations;    // unassigned-agent starts

    Instance() = default;
    // .mapfua input.
    Instance(const string& mapfua_fname);
    // MovingAI .map + .scen input for standard MAPF.
    Instance(const string& map_fname, const string& scen_fname, int num_agents);

    inline bool isObstacle(int loc) const { return my_map[loc]; }
    bool validMove(int curr, int next) const;
    list<int> getNeighbors(int curr) const;
    const vector<bool>& getMap() const { return my_map; }

    inline int linearizeCoordinate(int row, int col) const { return num_of_cols * row + col; }
    inline int getRowCoordinate(int id) const { return id / num_of_cols; }
    inline int getColCoordinate(int id) const { return id % num_of_cols; }
    inline int getManhattanDistance(int loc1, int loc2) const {
        return abs(getRowCoordinate(loc1) - getRowCoordinate(loc2)) +
               abs(getColCoordinate(loc1) - getColCoordinate(loc2));
    }

private:
    vector<bool> my_map;
    bool loadMapfua(const string& fname);
    bool loadMovingAIMap(const string& fname);
    bool loadMovingAIScen(const string& fname, int num_agents);
    int checkedLocation(int x, int y, const string& fname) const;
    void validateLocations(const string& fname) const;
};
