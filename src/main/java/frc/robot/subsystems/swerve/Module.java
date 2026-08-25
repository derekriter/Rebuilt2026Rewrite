// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.config.SwerveConfig;
import frc.robot.util.AlertUtils;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final IModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(IModuleIO io, int index) {
        this.io = io;
        this.index = index;
        driveDisconnectedAlert =
                // new Alert("Disconnected drive motor on module " + Integer.toString(index) + ".", AlertType.kError);
                AlertUtils.makeCANFailureAlert(SwerveConfig.modules[index].driveMotorName);
        turnDisconnectedAlert =
                // new Alert("Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
                AlertUtils.makeCANFailureAlert(SwerveConfig.modules[index].steerMotorName);
        turnEncoderDisconnectedAlert =
                // new Alert("Disconnected turn encoder on module " + Integer.toString(index) + ".", AlertType.kError);
                AlertUtils.makeCANFailureAlert(SwerveConfig.modules[index].encoderName);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(
                String.format(
                        "SwerveInputs/%d_%sModuleInputs", index, SwerveConfig.modules[index].prefix.toUpperCase()),
                inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters =
                    inputs.odometryDrivePositionsRad[i] * SwerveConfig.modules[index].constants.WheelRadius;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    public boolean isOperational() {
        return inputs.driveConnected && inputs.turnConnected && inputs.turnEncoderConnected;
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.turnPosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / SwerveConfig.modules[index].constants.WheelRadius);
        io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.kZero);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * SwerveConfig.modules[index].constants.WheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * SwerveConfig.modules[index].constants.WheelRadius;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }
}
