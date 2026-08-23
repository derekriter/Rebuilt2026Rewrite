// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IModuleIO {
    public static final IModuleIO blank = new IModuleIO() {
        @Override
        public void updateInputs(ModuleIOInputs inputs) {}

        @Override
        public void setDriveOpenLoop(double output) {}

        @Override
        public void setTurnOpenLoop(double output) {}

        @Override
        public void setDriveVelocity(double velocityRadPerSec) {}

        @Override
        public void setTurnPosition(Rotation2d rotation) {}
    };

    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ModuleIOInputs inputs);

    /** Run the drive motor at the specified open loop value. */
    public void setDriveOpenLoop(double output);

    /** Run the turn motor at the specified open loop value. */
    public void setTurnOpenLoop(double output);

    /** Run the drive motor at the specified velocity. */
    public void setDriveVelocity(double velocityRadPerSec);

    /** Run the turn motor to the specified rotation. */
    public void setTurnPosition(Rotation2d rotation);
}
