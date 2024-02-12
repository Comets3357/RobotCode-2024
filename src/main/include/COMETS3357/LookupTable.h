#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <set>
#include <string>
#include "COMETS3357/Configs/ConfigFiles.h"

namespace COMETS3357
{

    class LookupTable
    {
    public:
        LookupTable(std::string configName);

        LookupTableConfig config;

        std::set<std::pair<double, double>> lookupTable;

        double GetValue(double value);

    };

};