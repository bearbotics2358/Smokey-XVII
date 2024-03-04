#include <iostream>
#include <map>
#include <Prefs.h>

using namespace std;

class InterpolatingMap {
    public:
        void add(double distance, Values value);
        void remove(double distance);
        void clear();

        Values interpolate(double distance);
    private:
        Values lerp(double lower_bound, double upper_bound, double distance);
        Values values_add(Values A, Values B);
        Values scalar_multiply(Values A, double multiplier);

        map<double, Values> values_map;
};