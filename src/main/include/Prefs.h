
#pragma once // only add this code once; saves space by removing redundancy

#include <units/angle.h>
#include <units/length.h>
#include <frc/SerialPort.h>

// Declare constants such as CAN IDs here

/*======= For Interpolating Map =======*/
#define ANGLE_LOWER_BOUND 20.0
#define ANGLE_UPPER_BOUND 85.0

#define RPM_LOWER_BOUND 0.0
#define RPM_UPPER_BOUND 6000.0

/*======= DRIVER STATION & ITS RELATED STUFF CONSTANTS =======*/
#define COLLECTOR_BUTTON 3
#define INVERSE_COLLECTOR_BUTTON 4
#define SHOOTER_BUTTON 6
//#define DEFAULT_SHOOTER_ANGLE 36.0

// For the competition bot, this line *MUST* be enabled. For the practice bot, comment out this line.
#define COMP_BOT

#ifdef COMP_BOT  // The comp bot and the practice bot have some different IDs for various components
#define GYRO_ID 35
#else
#define GYRO_ID 35
#endif

#define PITCH_OFFSET -2;

#define AMP_BEAM_BREAK_PORT 5
#define COLLECTOR_BEAMBREAK_PORT 9

#define LIMIT_SWITCH 0 // change later

#define COLLECTOR_MOTOR_ID 30
#define SHOOTER_LEFT_MOTOR_ID 31
#define SHOOTER_RIGHT_MOTOR_ID 32
#define INDEXER_MOTOR_ID 33
#define PIVOT_MOTOR_ID 34
#define ARM_PIVOT_MOTOR_ID 35
#define CLIMBER_MOTOR_ID 36
#define EXTENSION_ID 37
#define ROLLER_ID 38


#define MAX_CLIMB_PERCENT 0.1
//in m/s
#define MAX_FREE_SPEED 4.968
#define MAX_ROT_SPEED 2.07


#define TICKS_STEERING 18.0 // roughly 18 "position" units per steering rotation

/*====== SWERVE MOTOR CONTROLLER IDS ======*/

/*

Module Numbering Scheme:

m = number engraved on module

    - Drive ID: 2m - 1
    - Steering ID: 2m
    - Encoder ID: 16 + m

*/

#ifdef COMP_BOT  // The comp bot and the practice bot have some different IDs for various components
#define FL_ID 6
#define FR_ID 1
#define BL_ID 7
#define BR_ID 5
#else
#define FL_ID 1
#define FR_ID 6
#define BL_ID 5
#define BR_ID 2
#endif


/*======= ENCODER CONSTANTS =======*/

// the distance we were getting from the wheel was not quite right, so we multiply them by this constant to get the right distance
#define DISTANCE_ADJUSTMANT_FACTOR 1.09789

// falcon encoder ticks per 1 revolution
#define FALCON_UNITS_PER_REV 2048

// swerve drive absolute encoder analog ports
#define FL_SWERVE_ABS_ENC_PORT 0
#define FR_SWERVE_ABS_ENC_PORT 3
#define BL_SWERVE_ABS_ENC_PORT 1
#define BR_SWERVE_ABS_ENC_PORT 2

// min and max voltage of absolute encoders on swerve drives
#define FL_SWERVE_ABS_ENC_MIN_VOLTS 0.014076
#define FL_SWERVE_ABS_ENC_MAX_VOLTS 4.952392

#define FR_SWERVE_ABS_ENC_MIN_VOLTS 0.037842
#define FR_SWERVE_ABS_ENC_MAX_VOLTS 4.962158

#define BL_SWERVE_ABS_ENC_MIN_VOLTS 0.004883
#define BL_SWERVE_ABS_ENC_MAX_VOLTS 4.641113

#define BR_SWERVE_ABS_ENC_MIN_VOLTS 0.010986
#define BR_SWERVE_ABS_ENC_MAX_VOLTS 4.963378

#define CANCODER_OFFSET_1 332.25
#define CANCODER_OFFSET_2 115.3
#define CANCODER_OFFSET_3 71.45
#define CANCODER_OFFSET_4 508.6 - 180 - 180
#define CANCODER_OFFSET_5 176.1 - 33.27
#define CANCODER_OFFSET_6 35.77
#define CANCODER_OFFSET_7 99.22
#define CANCODER_OFFSET_8 246.2
#define CANCODER_OFFSET_ARM -318.17


#define CANCODER_ID_1 17
#define CANCODER_ID_2 18
#define CANCODER_ID_3 19
#define CANCODER_ID_4 20
#define CANCODER_ID_5 21
#define CANCODER_ID_6 22
#define CANCODER_ID_7 23
#define CANCODER_ID_8 24
#define CANCODER_ID_ARM 25

static double CANCODER_OFFSETS[] = {
    CANCODER_OFFSET_1,
    CANCODER_OFFSET_2,
    CANCODER_OFFSET_3,
    CANCODER_OFFSET_4,
    CANCODER_OFFSET_5,
    CANCODER_OFFSET_6,
    CANCODER_OFFSET_7,
    CANCODER_OFFSET_8,
    CANCODER_OFFSET_ARM
    };

// @todo Measure this offset and update to the actual value
static const double ARM_ANGLE_OFFSET_DEGREES = 0.0;

/* ========== Shuttle constants ====== */

// Neo is 42 ticks / revolution, geared down, drive chain, ...
#define SHUTTLE_TICKS_PER_MM 2.7306

/* ========== Joystick Ports ========= */
#define OPERATOR_PORT 0
#define JOYSTICK_DEADZONE 0.15

#define DRIVER_PORT 5


/* ============ GEAR RATIOS ======== */
// I have a feeling this might be wrong, since our distance measurents are sligtly off
// ratio is drive motor rotations / wheel rotations
#define SWERVE_DRIVE_MOTOR_GEAR_RATIO (6.75 / 1.0)

#define SHOOTER_GEAR_RATIO (5.0)

#define INVERTED_MOTOR (-1.0) //used to allow for the inversion of motors in MK4i modules

// wheel diameter in meters
#define WHEEL_DIAMETER 0.095
#define EXTENSION_DIAMETER 0.0
#define CLIMBER_DIAMETER 0.0

// Vision Related
#define SPEAKER_HEIGHT 1.7021
#define RADIUS_OF_MOTOR 0.0254
#define VELOCITY_CONSTANT 1.01

/* ============= MqttClient ============= */

#define SEND_BUF_LEN 2048
#define RECV_BUF_LEN 2048


/* ============= CanHandler ============= */

#define LEFT_ARDUINO_CAN_ID 1
#define RIGHT_ARDUINO_CAN_ID 1

#define LEFT_ARDUINO_API_ID 2
#define RIGHT_ARDUINO_API_ID 3

#define FL_SWERVE_DATA_ID 0
#define BL_SWERVE_DATA_ID 1
#define FR_SWERVE_DATA_ID 2
#define BR_SWERVE_DATA_ID 3


/* ============= Vision ============= */

// the height of the camare used to track the target for shooting
constexpr units::length::meter_t TARGET_CAMERA_HEIGHT = units::length::meter_t(1.0);

// pitch of vision camera that tracks the target, positive is up
constexpr units::angle::radian_t TARGET_CAMERA_PITCH = units::angle::radian_t(1.0);

// the height of the target we are shooting at
constexpr units::length::meter_t TARGET_HEIGHT = units::length::meter_t(1.0);

// pitch of target
constexpr units::angle::radian_t TARGET_PITCH = units::angle::radian_t(0.0);

// mdns name of camera
#define SHOOTER_CAMERA_NAME "limelight1"

/* ============= Arduino ============= */

#define BAUD_RATE_ARDUINO 115200
#define USB_PORT_ARDUINO frc::SerialPort::kOnboard
#define DATA_BITS_ARDUINO 8
#define PARITY_ARDUINO frc::SerialPort::kParity_None
#define STOP_BITS_ARDUINO frc::SerialPort::kStopBits_One

#define ARDUINO_DIO_PIN 1


/*
sraight up: 78.57
"0": 130.34
back: 319.13
*/