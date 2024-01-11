#include "COMETS3357/LookupTable.h"

LookupTable::LookupTable()
{

}

void LookupTable::AddValues(std::set<std::pair<double, double>> table)
{
    lookupTable = table;
}

double LookupTable::GetValue(double value)
{

    double above = lookupTable.upper_bound({value, std::numeric_limits<double>::lowest()})->second;
    double below = std::prev(lookupTable.lower_bound({value, std::numeric_limits<double>::lowest()}))->second;

    return std::lerp(value, below, above);
}