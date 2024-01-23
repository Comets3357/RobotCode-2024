#include "COMETS3357/LookupTable.h"

using namespace COMETS3357;

LookupTable::LookupTable(std::string configName) : config{ConfigFiles::getInstance().GetConfigFiles().lookupTableConfigs[configName]}
{
    lookupTable = config.lookupTableData;
}

double LookupTable::GetValue(double value)
{

    std::pair<double, double> above = *lookupTable.upper_bound({value, std::numeric_limits<double>::lowest()});
    std::pair<double, double> below = *std::prev(lookupTable.lower_bound({value, std::numeric_limits<double>::lowest()}));
 
    double slope = (above.second - below.second)/(above.first - below.second);
    double angle = ((value - below.first) * slope) + below.second;
 
    return angle;

}