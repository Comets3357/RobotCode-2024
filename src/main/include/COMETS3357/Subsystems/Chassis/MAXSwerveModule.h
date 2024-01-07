// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include "COMETS3357/Configs/ConfigFiles.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxVelocity.h"
#include "COMETS3357/Subsystems/SparkMax/SparkMaxPosition.h"

namespace COMETS3357
{

    class MAXSwerveModule {
    public:
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    MAXSwerveModule(SwerveModuleConfig config);

    COMETS3357::SwerveModuleConfig configuration;

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    frc::SwerveModuleState GetState() ;

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    frc::SwerveModulePosition GetPosition() ;

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    void SetDesiredState(const frc::SwerveModuleState& state);

    /**
     * Zeroes all the SwerveModule encoders.
     */
    void ResetEncoders();

    /**
     * Runs every loop
     */
    void Periodic();

    private:

    COMETS3357::SparkMaxVelocity drivingMotor;
    COMETS3357::SparkMaxPosition azimuthMotor;

    double m_chassisAngularOffset = 0;
    frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                            frc::Rotation2d()};

    };

};