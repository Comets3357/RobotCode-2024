#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>

class LookupTable
{

    LookupTable();

    std::set<std::pair<double, double>> lookupTable;

    void AddValues(std::set<std::pair<double, double>> table);

    double GetValue(double value);

};