#pragma once
#include <Prefs.h>

class InterpolationValues {
    public:
        InterpolationValues();
        InterpolationValues(double a, double r);
        InterpolationValues(InterpolationValues const& obj);

        double getAngle();
        double getRPM();

        InterpolationValues operator+(InterpolationValues const& obj);
        InterpolationValues operator*(double multiplier);

        InterpolationValues clampValues();
    private:
        double clamp(double x, double lower, double upper);

        double angle;
   	    double rpm;
};