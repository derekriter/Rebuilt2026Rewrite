// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IGyroIO {
    public static final IGyroIO blank = new IGyroIO() {
        @Override
        public void updateInputs(GyroIOInputs inputs) {}
    };

    public static class GyroIOInputs implements LoggableInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocity_radps = 0.0;
        public double[] odometryYawTimestamps_s = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("connected", connected);
            table.put("yawPosition", yawPosition);
            table.put("yawVelocity", yawVelocity_radps, RadiansPerSecond.name());
            table.put("odometryYawTimestamps", odometryYawTimestamps_s);
            table.put("odometryYawPositions", odometryYawPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("connected", connected);
            yawPosition = table.get("yawPosition", yawPosition);
            yawVelocity_radps = table.get("yawVelocity", yawVelocity_radps);
            odometryYawTimestamps_s = table.get("odometryYawTimestamps", odometryYawTimestamps_s);
            odometryYawPositions = table.get("odometryYawPositions", odometryYawPositions);
        }
    }

    public void updateInputs(GyroIOInputs inputs);
}
