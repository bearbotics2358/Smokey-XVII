#pragma once

#include <Prefs.h>
#include <InterpolationValues.h>

using namespace std;

InterpolationValues::InterpolationValues() {
    angle = 20.0;
    rpm = 0.0;
}

// [parameter] = 0 means that when no parameter is passed in, it defaults to 0.
InterpolationValues::InterpolationValues(double a, double r) {
    angle = a;
    if (angle <= 20.0) {
        angle = 20.0;
    } else if (angle >= 85.0) {
        angle = 85.0;
    }
    
    rpm = r;
    if (rpm <= 0.0) {
        rpm = 0.0;
    } else if (rpm >= 6000.0) {
        rpm = 6000.0;
    }
}

InterpolationValues::InterpolationValues(InterpolationValues const& obj) {
    angle = obj.angle;
    rpm = obj.rpm;
}

double InterpolationValues::getAngle() {
    return angle;
}

double InterpolationValues::getRPM() {
    return rpm;
}

InterpolationValues InterpolationValues::operator+(InterpolationValues const& obj) {
    InterpolationValues res;
    
    res.angle = angle + obj.angle;
    res.rpm = rpm + obj.rpm;
    
    return res;
}

InterpolationValues InterpolationValues::operator*(double multiplier) {
    InterpolationValues res;
    
    res.angle = angle * multiplier;
    res.rpm = rpm * multiplier;
    
    return res;
}

InterpolationValues InterpolationValues::clampValues() {
    InterpolationValues res;
        
    res.angle = clamp(angle, ANGLE_LOWER_BOUND, ANGLE_UPPER_BOUND);
    res.rpm = clamp(rpm, RPM_LOWER_BOUND, RPM_UPPER_BOUND);
        
    return res;
}

double InterpolationValues::clamp(double x, double lower, double upper) {
    return min(upper, max(x, lower));
}