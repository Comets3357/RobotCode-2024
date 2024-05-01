#include "Subsystems/ShooterSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem(COMETS3357::SwerveSubsystem* swerveSubsystem, COMETS3357::GyroSubsystem* gyroSubsystem) : COMETS3357::Subsystem("ShooterSubsystem"), gyro{gyroSubsystem}, swerve{swerveSubsystem} {

}
    void ShooterSubsystem::Initialize()
    {
        
    }

void ShooterSubsystem::Periodic(){
    //frc::SmartDashboard::PutNumber("Velocity wheels", KickerWheel.GetRelativeVelocity());
    frc::SmartDashboard::PutNumber("PivotAbso", GetPivotAbsolutePosition());
    // SetPositionPivot(45);
    
    Pivot.Periodic();

    //frc::SmartDashboard::PutNumber("ANGLE LOOKUP", angleLookup.GetValue(2));
    //frc::SmartDashboard::PutNumber("Angle OFFSET ", offset); 

    

    if (turningTowardsTarget)
    {
        frc::Pose2d pos = swerve->GetPose();
    double distance2 = sqrt(pow((double)(targetPos.X() - pos.X()), 2) + pow((double)(targetPos.Y() - pos.Y()), 2)); 
    double shooterAngle2 = angleLookup.GetValue(distance2);
    double velocity2 = cos(shooterAngle2 * 3.14159 / 180) * 17; 

    frc::Pose2d robotPosition = swerve->GetMovingPose(distance2 / velocity2); 


        units::meter_t deltaX = robotPosition.X() - targetPos.X();
        units::meter_t deltaY = robotPosition.Y() - targetPos.Y();

        double angle = atan2((double)deltaY, (double)deltaX);
        
        turnToPID.SetP(0.8);
        swerve->overrideRotation = units::radians_per_second_t{std::clamp(turnToPID.Calculate((-gyro->m_navx.GetAngle() * 3.14159 / 180) + gyro->angleOffset + ((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) ? 3.14159 : 0) , angle), -0.6, 0.6)};//rotationPLookup.GetValue(0);

  
    }
}

void ShooterSubsystem::SetVelocityFlyWheel(double velocity)
{
    FlyWheel.SetVelocity(velocity);
}
void ShooterSubsystem::SetVelocityFlyWheel(std::string velocity)
{
    FlyWheel.SetVelocity(velocity);
}

void ShooterSubsystem::SetVelocityKickerWheel(double velocity)
{
    KickerWheel.SetVelocity(velocity);
}

void ShooterSubsystem::SetVelocityKickerWheel(std::string velocity)
{
    KickerWheel.SetVelocity(velocity);
}

// Manual // 

void ShooterSubsystem::SetPercentFlyWheel(double percent) 
{
    FlyWheel.SetPercent(percent);
}

void ShooterSubsystem::SetPercentKickerWheel (double percent)
{
    KickerWheel.SetPercent(percent);
}

double ShooterSubsystem::GetVelocityKickerWheel()
{
    return KickerWheel.GetRelativeVelocity();
}

double ShooterSubsystem::GetVelocityFlyWheel()
{
    return FlyWheel.GetRelativeVelocity();
}

void ShooterSubsystem::SetPercentPivot(double percent) 
{
    Pivot.SetPower(percent);
}

void ShooterSubsystem::SetPositionPivot(double position)
{
    Pivot.SetPosition(position, offset); 
}

void ShooterSubsystem::SetPositionPivot(std::string position)
{
    Pivot.SetPosition(position, offset); 
}


double ShooterSubsystem::GetPivotRelativePosition()
{
    return Pivot.GetRelativePosition();
}

double ShooterSubsystem::GetPivotAbsolutePosition()
{
    frc::SmartDashboard::PutNumber("Pivot Abs", pivotAbsoluteEncoder.GetOutput());
    return ((double)pivotAbsoluteEncoder.GetOutput() * 360) - 237.0;
}

std::pair<double, double> ShooterSubsystem::calculateDistanceTravelled(std::pair<double, double> velocity, double time)
{
    double x, y;
    x = velocity.first * (time + .1);
    y = velocity.second * (time + .1);
    return {x,y};
}

std::pair<double, double> ShooterSubsystem::calculateFinalPosition(std::pair<double, double> initial, std::pair<double, double> travel)
{
    return {travel.first - initial.first, travel.second - initial.first};
}

void ShooterSubsystem::startTurnToTarget()
{
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
        targetPos = frc::Translation2d{units::meter_t{16.579}, units::meter_t{5.547867999999999}};
    }
    else
    {
        targetPos = frc::Translation2d{units::meter_t{0}, units::meter_t{5.547867999999999}};
    }
    turnToPID.EnableContinuousInput(-3.14159 + gyro->angleOffset, 3.14159 + gyro->angleOffset);
    turningTowardsTarget = true;
    swerve->controllingSwerveRotation = false;
    
}

void ShooterSubsystem::startTurnPassZone()
{
    frc::Pose2d robotPose = swerve->GetPose();
    xOffset++;
    if (xOffset > 2) xOffset = 0;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
    {
  
        if ((double)robotPose.X() > 5)
        {
            targetPos = frc::Translation2d{units::meter_t{14.66}, units::meter_t{6.86}};
        }
        else
        {
            targetPos = frc::Translation2d{units::meter_t{9}, units::meter_t{7}};
        }
        targetPos = frc::Translation2d{units::meter_t{(double)targetPos.X() + (0.4 * xOffset)}, units::meter_t{(double)targetPos.Y()}};
    }
    else
    {
        if ((double)robotPose.X() < 12)
        {
            targetPos = frc::Translation2d{units::meter_t{1.91}, units::meter_t{6.86}};
        }
        else
        {
            targetPos = frc::Translation2d{units::meter_t{7.61}, units::meter_t{7}};
        }
        targetPos = frc::Translation2d{units::meter_t{(double)targetPos.X() - (0.4 * xOffset)}, units::meter_t{(double)targetPos.Y()}};
    }
    turnToPID.EnableContinuousInput(-3.14159 + gyro->angleOffset, 3.14159 + gyro->angleOffset);
    turningTowardsTarget = true;
    swerve->controllingSwerveRotation = false;
    
}

void ShooterSubsystem::stopTurnToTarget()
{
    turningTowardsTarget = false;
    swerve->controllingSwerveRotation = true;
    swerve->overrideRotation = units::radians_per_second_t{0};
}