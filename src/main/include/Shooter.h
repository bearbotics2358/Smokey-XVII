#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <ctre/Phoenix.h>

class Shooter{
    public:
    Shooter(int rightShooterMotorID, int leftShooterMotorID, int pivotMotorID);
    void setSpeed(double percent);
    void stopShooter();
    private:
    // rev::CANSparkMax rightShooterMotor;
    // rev::CANSparkMax leftShooterMotor;
    frc2::PIDController leftShooterPID;
    frc2::PIDController rightShooterPID;

    TalonFX rightShooterMotor;
    TalonFX leftShooterMotor;
    TalonFX pivotMotor;
    TalonFXSensorCollection pivotEncoder;
};