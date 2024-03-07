#pragma once

#include <iostream>
#include <map>
#include <Prefs.h>
#include <InterpolatingMap.h>

using namespace std;

void InterpolatingMap::insert(double distance, InterpolationValues value) {
    values_map[distance] = value;
}

void InterpolatingMap::remove(double distance) {
    values_map.erase(distance);
}

void InterpolatingMap::clear() {
    values_map.clear();
}

InterpolationValues InterpolatingMap::operator[](double distance) {
    // less than min distance
    double curr_min = values_map.begin()->first;
    if (distance < curr_min) {
        return( values_map[curr_min] * (distance / curr_min)).clampValues();
    }
    
    // greater than max distance
    double curr_max = values_map.rbegin()->first;
    if (distance > curr_max) {
        return (values_map[curr_max] * (distance / curr_max)).clampValues();
    }
    
    // find lower and upper bound
    InterpolationValues res;
    for (auto i = values_map.begin(); i != prev(values_map.end()); i++) {
        if (distance <= next(i)->first) {
            res = lerp(i->first, next(i)->first, distance).clampValues();
        }
    }
    return res;
}

InterpolationValues InterpolatingMap::lerp(double lowerBound, double upperBound, double distance) {
    // All it's doing is A + (B-A) * t, where A is lower_bound & B is upper_bound
    double t = (distance - lowerBound) / (upperBound - lowerBound);

    InterpolationValues res = values_map[upperBound] + values_map[lowerBound] * -1.0;
    res = res * t;
    res = res + values_map[lowerBound];
        
    return res;
}