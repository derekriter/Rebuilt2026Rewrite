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
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.IModuleIO.ModuleIOInputs;
import frc.robot.util.AlertUtils;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final IModuleIO io;
    private final ModuleIOInputs inputs = new ModuleIOInputs();
    private final int index;
    private final String moduleName;
    private final SwerveConstants.ModuleInfo config;

    private final Alert driveCANAlert;
    private final Alert driveBreakerAlert;
    private boolean driveConnectedLast = false;
    private boolean driveBreaker = false;
    private boolean driveBreakerLast = false;

    private final Alert steerCANAlert;
    private final Alert steerBreakerAlert;
    private boolean steerConnectedLast = false;
    private boolean steerBreaker = false;
    private boolean steerBreakerLast = false;

    private final Alert encoderCANAlert;
    private final Alert encoderBreakerAlert;
    private boolean encoderConnectedLast = false;
    private boolean encoderBreaker = false;
    private boolean encoderBreakerLast = false;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(IModuleIO _io, int _index) {
        io = _io;
        index = _index;
        config = SwerveConstants.modules[index];
        moduleName = String.format("%d_%sModule", _index, config.prefix.toUpperCase());

        driveCANAlert = AlertUtils.makeCANFailureAlert(config.driveMotorName);
        driveBreakerAlert = AlertUtils.makeBreakerTripAlert(config.driveMotorName);
        steerCANAlert = AlertUtils.makeCANFailureAlert(config.steerMotorName);
        steerBreakerAlert = AlertUtils.makeBreakerTripAlert(config.steerMotorName);
        encoderCANAlert = AlertUtils.makeCANFailureAlert(config.encoderName);
        encoderBreakerAlert = AlertUtils.makeBreakerTripAlert(config.encoderName);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("SwerveInputs/" + moduleName + "Inputs", inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps_s.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositions_rad[i] * config.constants.WheelRadius;
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update logging
        driveBreaker = RobotContainer.instance().pdh.isBreakerTripped(config.driveChannelID);
        Logger.recordOutput(
                String.format("CAN/%s_%d", config.driveMotorName, config.driveCANID), inputs.driveConnected);
        driveCANAlert.set(!inputs.driveConnected && !driveBreaker);
        if (inputs.driveConnected != driveConnectedLast) {
            if (inputs.driveConnected) {
                Console.reportCANConnect(config.driveMotorName, config.driveCANID, config.driveChannelID);
            } else {
                Console.reportCANDisconnect(config.driveMotorName, config.driveCANID, config.driveChannelID);
            }
        }
        driveBreakerAlert.set(driveBreaker);
        if (driveBreaker != driveBreakerLast) {
            if (driveBreaker) {
                Console.reportBreakerTrip(config.driveMotorName, config.driveCANID, config.driveChannelID);
            } else {
                Console.reportBreakerReset(config.driveMotorName, config.driveCANID, config.driveChannelID);
            }
        }

        steerBreaker = RobotContainer.instance().pdh.isBreakerTripped(config.steerChannelID);
        Logger.recordOutput(
                String.format("CAN/%s_%d", config.steerMotorName, config.steerCANID), inputs.steerConnected);
        steerCANAlert.set(!inputs.steerConnected && !steerBreaker);
        if (inputs.steerConnected != steerConnectedLast) {
            if (inputs.steerConnected) {
                Console.reportCANConnect(config.steerMotorName, config.steerCANID, config.steerChannelID);
            } else {
                Console.reportCANDisconnect(config.steerMotorName, config.steerCANID, config.steerChannelID);
            }
        }
        steerBreakerAlert.set(steerBreaker);
        if (steerBreaker != steerBreakerLast) {
            if (steerBreaker) {
                Console.reportBreakerTrip(config.steerMotorName, config.steerCANID, config.steerChannelID);
            } else {
                Console.reportBreakerReset(config.steerMotorName, config.steerCANID, config.steerChannelID);
            }
        }

        encoderBreaker = RobotContainer.instance().pdh.isBreakerTripped(config.encoderChannelID);
        Logger.recordOutput(
                String.format("CAN/%s_%d", config.encoderName, config.encoderCANID), inputs.encoderConnected);
        encoderCANAlert.set(!inputs.encoderConnected && !encoderBreaker);
        if (inputs.encoderConnected != encoderConnectedLast) {
            if (inputs.encoderConnected) {
                Console.reportCANConnect(config.encoderName, config.encoderCANID, config.encoderChannelID);
            } else {
                Console.reportCANDisconnect(config.encoderName, config.encoderCANID, config.encoderChannelID);
            }
        }
        encoderBreakerAlert.set(encoderBreaker);
        if (encoderBreaker != encoderBreakerLast) {
            if (encoderBreaker) {
                Console.reportBreakerTrip(config.encoderName, config.encoderCANID, config.encoderChannelID);
            } else {
                Console.reportBreakerReset(config.encoderName, config.encoderCANID, config.encoderChannelID);
            }
        }

        driveConnectedLast = inputs.driveConnected;
        driveBreakerLast = driveBreaker;
        steerConnectedLast = inputs.steerConnected;
        steerBreakerLast = steerBreaker;
        encoderConnectedLast = inputs.encoderConnected;
        encoderBreakerLast = encoderBreaker;
    }

    public boolean isOperational() {
        return (inputs.driveConnected && !driveBreaker)
                && (inputs.steerConnected && !steerBreaker)
                && (inputs.encoderConnected && !encoderBreaker);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.steerPosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / config.constants.WheelRadius);
        io.setSteerPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output_V) {
        io.setDriveOpenLoop(output_V);
        io.setSteerPosition(Rotation2d.kZero);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setSteerOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.steerPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPosition_m() {
        return inputs.drivePosition_rad * config.constants.WheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocity_mps() {
        return inputs.driveVelocity_radps * config.constants.WheelRadius;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPosition_m(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity_mps(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps_s() {
        return inputs.odometryTimestamps_s;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition_rad() {
        return inputs.drivePosition_rad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity_rps() {
        return Units.radiansToRotations(inputs.driveVelocity_radps);
    }
}
