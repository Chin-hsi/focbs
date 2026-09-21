#include "conflict.h"

std::ostream& operator<<(std::ostream& os, const Constraint& c) {
    os << "<" << get<0>(c) << "," << get<1>(c) << ","
       << get<2>(c) << "," << get<3>(c) << ",";
    switch (get<4>(c)) {
        case constraint_type::VERTEX:    os << "V"; break;
        case constraint_type::EDGE:      os << "E"; break;
        case constraint_type::LEQLENGTH: os << "L"; break;
        case constraint_type::GLENGTH:   os << "G"; break;
        default: break;
    }
    os << ">";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    os << "conflict: " << c.a1 << " with ";
    for (auto& con : c.constraint1) os << con << ",";
    os << " and " << c.a2 << " with ";
    for (auto& con : c.constraint2) os << con << ",";
    return os;
}
