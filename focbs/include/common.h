#pragma once
#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <map>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cassert>
#include <climits>
#include <cstdlib>
#include <memory>
#include <queue>
#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <boost/heap/pairing_heap.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using std::vector;
using std::list;
using std::set;
using std::map;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::ofstream;

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry {
    int location = -1;
    PathEntry(int loc = -1) : location(loc) {}
};

typedef vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);
bool isSamePath(const Path& p1, const Path& p2);
