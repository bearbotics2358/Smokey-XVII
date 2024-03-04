#include <InterpolatingMap.h>
#include <Prefs.h>

using namespace std;

void InterpolatingMap::add(double distance, Values value) {
    values_map[distance] = value;
}

void InterpolatingMap::remove(double distance) {
    values_map.erase(distance);
}

void InterpolatingMap::clear() {
    values_map.clear();
}

Values InterpolatingMap::interpolate(double distance) {
    // add bounds checking
    for (auto i = values_map.begin(); i != prev(values_map.end()); i++) {
        if (distance <= next(i)->first) {
            return lerp(i->first, next(i)->first, distance);
        }
    }
}

Values InterpolatingMap::lerp(double lower_bound, double upper_bound, double distance) {
    double t = (distance - lower_bound) /(upper_bound - lower_bound);

    Values res = values_add(values_map[upper_bound], scalar_multiply(values_map[lower_bound], -1));
    res = scalar_multiply(res, t);
    res = values_add(values_map[lower_bound], res);

    return res;
}

Values InterpolatingMap::values_add(Values A, Values B) {
    Values res;

    res.angle = A.angle + B.angle;
    res.rpm = A.rpm + B.rpm;

    return res;
}

Values InterpolatingMap::scalar_multiply(Values A, double multiplier) {
    Values res;

    res.angle = A.angle * multiplier;
    res.rpm = A.rpm * multiplier;

    return res;
}