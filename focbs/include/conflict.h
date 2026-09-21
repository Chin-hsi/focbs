#pragma once
#include "common.h"

enum constraint_type { VERTEX, EDGE, LEQLENGTH, GLENGTH, CONSTRAINT_COUNT };

// <agent, loc/from, -1/to, t, type>
typedef std::tuple<int, int, int, int, constraint_type> Constraint;
std::ostream& operator<<(std::ostream& os, const Constraint& constraint);

class Conflict {
public:
    int a1;
    int a2;
    list<Constraint> constraint1;
    list<Constraint> constraint2;

    void vertexConflict(int a1, int a2, int v, int t) {
        constraint1.clear(); constraint2.clear();
        this->a1 = a1; this->a2 = a2;
        constraint1.emplace_back(a1, v, -1, t, constraint_type::VERTEX);
        constraint2.emplace_back(a2, v, -1, t, constraint_type::VERTEX);
    }

    void edgeConflict(int a1, int a2, int v1, int v2, int t) {
        constraint1.clear(); constraint2.clear();
        this->a1 = a1; this->a2 = a2;
        constraint1.emplace_back(a1, v1, v2, t, constraint_type::EDGE);
        constraint2.emplace_back(a2, v2, v1, t, constraint_type::EDGE);
    }
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);
