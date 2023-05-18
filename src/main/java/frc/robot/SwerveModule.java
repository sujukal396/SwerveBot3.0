// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class SwerveModule {
    private static final double wheelRadius = 2;
    private static final double encoderResolution = 2048.0;

    private static final double moduleMaxAngularVelocity = DrivetrainSubsystem.maxAngularSpeed;
    private static final double moduleMaxAngularAcceleration = 2 * Math.PI;
    
    private TalonFX driveMotor;
    private TalonFX turnMotor;

    private final PIDController driveController = new PIDController(1, 0, 0); // dummy values plz change later

    private final ProfiledPIDController turnController = new ProfiledPIDController(
                                                                                0, 
                                                                                0,
                                                                                0,
                                                                                new TrapezoidProfile.Constraints(moduleMaxAngularVelocity, moduleMaxAngularAcceleration));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3); // idk what happens here
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5); // just trusting in wpilib

    public SwerveModule(int driveMotorID, int turnMotorID) {
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);

        /*
         * wpi had code to set up encoders
         * idk what we need to do for falcons
         * that is a thursday problem
         * - sujit
         */

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        // this code is adapted from the wpilib example code to work with falcons
        // will need to check if it actually works
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(turnMotor.getSelectedSensorPosition()));
    }

    public SwerveModulePosition getPosition() {
        // same story here
        return new SwerveModulePosition(driveMotor.getSelectedSensorPosition(), new Rotation2d(turnMotor.getSelectedSensorPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // funi optimizaiton
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turnMotor.getSelectedSensorPosition()));


        final double driveOutput = driveController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);
        final double driveFF = driveFeedforward.calculate(state.speedMetersPerSecond);


        final double turnOutput = turnController.calculate(turnMotor.getSelectedSensorPosition(), state.angle.getRadians());
        final double turnFF = turnFeedforward.calculate(turnController.getSetpoint().velocity);

        // honestly, im just going off of wpilib
        driveMotor.set(ControlMode.Current, driveOutput + driveFF);
        turnMotor.set(ControlMode.Current, turnOutput + turnFF);
    }

}
