#include "COMETS3357/Subsystems/Chassis/DrivebaseSubsystem.h"

using namespace COMETS3357;

DrivebaseSubsystem::DrivebaseSubsystem()
{
    dbFR.motor.Follow(dbR.motor);
    dbFL.motor.Follow(dbL.motor);

    dbR.motor.SetSmartCurrentLimit(60);
    dbL.motor.SetSmartCurrentLimit(60);
    dbFR.motor.SetSmartCurrentLimit(60);
    dbFL.motor.SetSmartCurrentLimit(60);

    dbR.motor.SetInverted(true);
    dbL.motor.SetInverted(false);
    dbFR.motor.SetInverted(true);
    dbFL.motor.SetInverted(false);
    
    pid.FF = 1;
    pid.P = 1;

    SetVelocityPID(pid);


}

void DrivebaseSubsystem::SetTicksToMetersConversion(double wheelDiameterInches, double gearRatio)
{

}

void DrivebaseSubsystem::SetTicksToMetersConversion(double conversion)
{

}

void DrivebaseSubsystem::SetVelocityPID(PID pid)
{
    dbR.SetVelocityPID(pid);
    dbL.SetVelocityPID(pid);
    dbFR.SetVelocityPID(pid);
    dbFL.SetVelocityPID(pid);
}

void DrivebaseSubsystem::SetVelocity(double left, double right)
{
    dbR.SetVelocity(right);
    dbL.SetVelocity(left);
    dbFR.SetVelocity(right);
    dbFL.SetVelocity(left);
}

void DrivebaseSubsystem::SetPercent(double left, double right)
{
    dbR.SetPercent(right);
    dbL.SetPercent(left);
    dbFR.SetPercent(right);
    dbFL.SetPercent(left);
}

void DrivebaseSubsystem::ResetOdometry(double x, double y, double radians, double gyroYaw) 
{
    const units::meter_t meterX{x};
    const units::meter_t meterY{y};

    const units::radian_t radianYaw{radians};
    // frc::SmartDashboard::PutNumber("Pi", pi);
    // frc::SmartDashboard::PutNumber("radian yaw", robotData.gyroData.rawYaw / 180 * pi);

    const units::radian_t gyroRadians{gyroYaw / 180 * M_PI};
    // frc::SmartDashboard::PutNumber("RORaw Yaw", robotData.gyroData.rawYaw);

    const frc::Rotation2d gyroRotation{gyroRadians};
    const frc::Pose2d resetPose{meterX, meterY, radianYaw};
    // zeroEncoders();
    odometry.ResetPosition(gyroRotation, units::meter_t{getEncoderDistance(dbL.GetRelativePosition())}, units::meter_t{getEncoderDistance(dbR.GetRelativePosition())},  resetPose);

    //zeroEncoders();
}

double DrivebaseSubsystem::getEncoderDistance(double encoderPosition)
{
    return encoderPosition*rotationsToMeters;
}

void DrivebaseSubsystem::resetOdometry(const frc::Pose2d &pose, double gyroAngle) 
{
    const units::radian_t gyroRadians{gyroAngle};
    frc::Rotation2d gyroRotation{gyroRadians};

    odometry.ResetPosition(gyroRotation, units::meter_t{getEncoderDistance(dbL.GetRelativePosition())}, units::meter_t{getEncoderDistance(dbR.GetRelativePosition())},  pose);
    //zeroEncoders();
}