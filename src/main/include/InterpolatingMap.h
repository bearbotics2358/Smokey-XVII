#pragma once

#include <iostream>
#include <map>
#include <Prefs.h>
#include <InterpolationValues.h>

using namespace std;

class InterpolatingMap {
    public:
        void insert(double distance, InterpolationValues value);
        void remove(double distance);
        void clear();

        InterpolationValues interpolate(double distance);
        InterpolationValues operator[](double distance);
    private:
        InterpolationValues lerp(double lower_bound, double upper_bound, double distance);

        map<double, InterpolationValues> values_map;
};