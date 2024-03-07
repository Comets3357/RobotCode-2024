



#include "COMETS3357/Auton/AdvancedAutoPathCommand.h"

AdvancedAutonPathCommand::AdvancedAutonPathCommand(COMETS3357::SwerveSubsystem* swerveSubsystem, double rotSpeed, double moveSpeed, frc::Pose2d pose) 
{
    swerve = swerveSubsystem;
    translatePID.SetTolerance(0.1);
    rotPID.SetTolerance(0.1);
    translatePID.SetP(0.75);
    rotPID.SetP(0.6);
    rot = rotSpeed;
    speed = moveSpeed;
    targetPose = pose;
    rotPID.SetSetpoint((double)targetPose.Rotation().Radians());
    rotPID.EnableContinuousInput(-3.14159265, 3.14159265);
    AddRequirements({swerve}); 
}

void AdvancedAutonPathCommand::Initialize()
{

}

void AdvancedAutonPathCommand::Execute()
{
    lowestIntersectObstacle = -1;
    lowestIntersectDistance = 100_m;
    frc::Pose2d currentPose = swerve->GetPose2();
    
    double slope = ((double)currentPose.Y() - (double)targetPose.Y())/((double)currentPose.X() - (double)targetPose.X());
    double inverseSlope = -((double)currentPose.X() - (double)targetPose.X())/((double)currentPose.Y() - (double)targetPose.Y());

    for (int i = 0; i < trussPositions.size(); i++)
    {
        std::optional<frc::Translation2d> intersectionPoint = CheckIntersection(currentPose.Translation(), targetPose.Translation(), trussPositions[i], radiusLimit);
        if (intersectionPoint.has_value() && intersectionPoint.value().Distance(currentPose.Translation()) < lowestIntersectDistance)
        {
            lowestIntersectDistance = intersectionPoint.value().Distance(currentPose.Translation());
            lowestIntersectPos = intersectionPoint.value();
            lowestIntersectObstacle = i;
        }
    }

    if (lowestIntersectObstacle != -1)
    {
        frc::Translation2d relativeToObs{lowestIntersectPos.value() - trussPositions[lowestIntersectObstacle]};
        units::meter_t ratio = radiusLimit / relativeToObs.Distance(frc::Translation2d{0_m, 0_m});
        frc::Translation2d tangentPos = frc::Translation2d{(relativeToObs.X() * ratio) + trussPositions[lowestIntersectObstacle].X(), (relativeToObs.Y() * ratio) + trussPositions[lowestIntersectObstacle].Y()};


        double rotationSpeed = std::clamp(rotPID.Calculate((double)currentPose.Rotation().Radians()), -rot, rot);
        double angle = atan2((double)currentPose.X() - (double)tangentPos.X(), (double)currentPose.Y() - (double)tangentPos.Y());
        double robotSpeed = std::clamp(translatePID.Calculate((double)currentPose.Translation().Distance(targetPose.Translation()), 0), -speed, speed);
        double movementX = sin(angle) * robotSpeed;
        double movementY = cos(angle) * robotSpeed;//std::clamp(translatePID.Calculate((double)currentPose.Y(), (double)targetPose.Y()), -speed, speed);
        swerve->Drive(units::meters_per_second_t{movementX}, units::meters_per_second_t{movementY}, units::radians_per_second_t{rotationSpeed}, true, true, &swerve->kDriveKinematics);
        
    }
    else
    {   
        double rotationSpeed = std::clamp(rotPID.Calculate((double)currentPose.Rotation().Radians()), -rot, rot);
        double angle = atan2((double)currentPose.X() - (double)targetPose.X(), (double)currentPose.Y() - (double)targetPose.Y());
        double robotSpeed = std::clamp(translatePID.Calculate((double)targetPose.Translation().Distance(targetPose.Translation()), 0), -speed, speed);
        double movementX = sin(angle) * robotSpeed;
        double movementY = cos(angle) * robotSpeed;//std::clamp(translatePID.Calculate((double)currentPose.Y(), (double)targetPose.Y()), -speed, speed);
        swerve->Drive(units::meters_per_second_t{movementX}, units::meters_per_second_t{movementY}, units::radians_per_second_t{rotationSpeed}, true, true, &swerve->kDriveKinematics);
    }

}

std::optional<frc::Translation2d> AdvancedAutonPathCommand::CheckIntersection(frc::Translation2d startPos, frc::Translation2d endPos, frc::Translation2d obstaclePos, units::meter_t rad)
{
    // Calculate the vector from the start point to the center of the circle
    frc::Translation2d startToCenter{obstaclePos.X() - startPos.X(), obstaclePos.Y() - startPos.Y()};

    // Calculate the vector representing the line segment
    frc::Translation2d lineVector{endPos.X() - startPos.X(), endPos.Y() - startPos.Y()};

    // Project the start-to-center vector onto the line segment
    double t = ((double)startToCenter.X() * (double)lineVector.X() + (double)startToCenter.Y() * (double)lineVector.Y()) / 
               ((double)lineVector.X() * (double)lineVector.X() + (double)lineVector.Y() + (double)lineVector.Y());

    t = std::clamp(t, 0.0, 1.0);

    frc::Translation2d intersectionPoint{startPos.X() + units::meter_t{t} * lineVector.X(), startPos.Y() + units::meter_t{t} * lineVector.Y()};

    if (intersectionPoint.Distance(obstaclePos) <= rad)
    {
        return intersectionPoint;
    }
    else
    {
        return std::nullopt; // no intersection
    }

}

bool AdvancedAutonPathCommand::IsFinished()
{
    frc::Pose2d currentPose = swerve->GetPose2();
    if (currentPose.Translation().Distance(targetPose.Translation()) < units::meter_t{0.1} && rotPID.AtSetpoint())
    {
            swerve->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerve->kDriveKinematics);

return true;
    }
    return false;
}

void AdvancedAutonPathCommand::End(bool interrupted)
{
    swerve->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, true, &swerve->kDriveKinematics);
}