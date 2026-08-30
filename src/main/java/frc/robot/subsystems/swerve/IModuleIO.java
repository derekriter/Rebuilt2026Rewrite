// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IModuleIO {
    public static final IModuleIO blank = new IModuleIO() {
        @Override
        public void updateInputs(ModuleIOInputs inputs) {}

        @Override
        public void setDriveOpenLoop(double output) {}

        @Override
        public void setSteerOpenLoop(double output) {}

        @Override
        public void setDriveVelocity(double velocityRadPerSec) {}

        @Override
        public void setSteerPosition(Rotation2d rotation) {}
    };

    public static class ModuleIOInputs implements LoggableInputs {
        public boolean driveConnected = false;
        public double drivePosition_rad = 0;
        public double driveVelocity_radps = 0;
        public double driveTemp_C = 0;
        public double driveAppliedVoltage_V = 0;
        public double driveStatorCurrent_A = 0;
        public double driveSupplyCurrent_A = 0;

        public boolean steerConnected = false;
        public Rotation2d steerPosition = Rotation2d.kZero;
        public double steerVelocity_radps = 0;
        public double steerTemp_C = 0;
        public double steerAppliedVoltage_V = 0;
        public double steerStatorCurrent_A = 0;
        public double steerSupplyCurrent_A = 0;

        public boolean encoderConnected = false;
        public Rotation2d encoderAbsolutePosition = Rotation2d.kZero;

        public double[] odometryTimestamps_s = new double[] {};
        public double[] odometryDrivePositions_rad = new double[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("driveConnected", driveConnected);
            table.put("drivePosition", drivePosition_rad, Radians.name());
            table.put("driveVelocity", driveVelocity_radps, RadiansPerSecond.name());
            table.put("driveTemp", driveTemp_C, Celsius.name());
            table.put("driveAppliedVoltage", driveAppliedVoltage_V, Volts.name());
            table.put("driveStatorCurrent", driveStatorCurrent_A, Amps.name());
            table.put("driveSupplyCurrent", driveSupplyCurrent_A, Amps.name());

            table.put("steerConnected", steerConnected);
            table.put("steerPosition", steerPosition);
            table.put("steerVelocity", steerVelocity_radps, RadiansPerSecond.name());
            table.put("steerTemp", steerTemp_C, Celsius.name());
            table.put("steerAppliedVoltage", steerAppliedVoltage_V, Volts.name());
            table.put("steerStatorCurrent", steerStatorCurrent_A, Amps.name());
            table.put("steerSupplyCurrent", steerSupplyCurrent_A, Amps.name());

            table.put("encoderConnected", encoderConnected);
            table.put("encoderAbsolutePosition", encoderAbsolutePosition);

            table.put("odometryTimestamps", odometryTimestamps_s);
            table.put("odometryDrivePositions", odometryDrivePositions_rad);
            table.put("odometrySteerPositons", odometrySteerPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            driveConnected = table.get("driveConnected", driveConnected);
            drivePosition_rad = table.get("drivePosition", drivePosition_rad);
            driveVelocity_radps = table.get("driveVelocity", driveVelocity_radps);
            driveTemp_C = table.get("driveTemp", driveTemp_C);
            driveAppliedVoltage_V = table.get("driveAppliedVoltage", driveAppliedVoltage_V);
            driveStatorCurrent_A = table.get("driveStatorCurrent", driveStatorCurrent_A);
            driveSupplyCurrent_A = table.get("driveSupplyCurrent", driveSupplyCurrent_A);

            steerConnected = table.get("steerConnected", steerConnected);
            steerPosition = table.get("steerPosition", steerPosition);
            steerVelocity_radps = table.get("steerVelocity", steerVelocity_radps);
            steerTemp_C = table.get("steerTemp", steerTemp_C);
            steerAppliedVoltage_V = table.get("steerAppliedVoltage", steerAppliedVoltage_V);
            steerStatorCurrent_A = table.get("steerStatorCurrent", steerStatorCurrent_A);
            steerSupplyCurrent_A = table.get("steerSupplyCurrent", steerSupplyCurrent_A);

            encoderConnected = table.get("encoderConnected", encoderConnected);
            encoderAbsolutePosition = table.get("encoderAbsolutePosition", encoderAbsolutePosition);

            odometryTimestamps_s = table.get("odometryTimestamps", odometryTimestamps_s);
            odometryDrivePositions_rad = table.get("odometryDrivePositions", odometryDrivePositions_rad);
            odometrySteerPositions = table.get("odometrySteerPositons", odometrySteerPositions);
        }
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ModuleIOInputs inputs);

    /** Run the drive motor at the specified open loop value. */
    public void setDriveOpenLoop(double output_V);

    /** Run the turn motor at the specified open loop value. */
    public void setSteerOpenLoop(double output_V);

    /** Run the drive motor at the specified velocity. */
    public void setDriveVelocity(double vel_radps);

    /** Run the turn motor to the specified rotation. */
    public void setSteerPosition(Rotation2d rotation);
}
