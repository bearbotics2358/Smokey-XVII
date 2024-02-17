#pragma once

#include "Prefs.h"
#include <frc/DigitalInput.h>

/** Class for a beam break.
 *  A beam break is a sensor that shoots an infrared beam across space, and it can detect when the beam is broken by an object.
 */
class BeamBreak {
    public:
        BeamBreak(int port);
        bool beamBroken();
    private:
        bool isBroken = false;
        frc::DigitalInput a_Input;
};