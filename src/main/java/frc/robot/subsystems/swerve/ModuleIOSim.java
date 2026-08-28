// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SwerveConstants;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements IModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double STEER_KP = 8.0;
    private static final double STEER_KD = 0.0;
    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor STEER_GEARBOX = DCMotor.getFalcon500Foc(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim steerSim;

    private boolean driveClosedLoop = false;
    private boolean steerClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController steerController = new PIDController(STEER_KP, 0, STEER_KD);
    private double driveFFVoltage_V = 0.0;
    private double driveAppliedVoltage_V = 0.0;
    private double steerAppliedVoltage_V = 0.0;

    public ModuleIOSim(int index) {
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants =
                SwerveConstants.modules[index].constants;

        // Create drive and turn sim models
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
                DRIVE_GEARBOX);
        steerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        STEER_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
                STEER_GEARBOX);

        // Enable wrapping for turn PID
        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVoltage_V =
                    driveFFVoltage_V + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (steerClosedLoop) {
            steerAppliedVoltage_V = steerController.calculate(steerSim.getAngularPositionRad());
        } else {
            steerController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVoltage_V, -12.0, 12.0));
        steerSim.setInputVoltage(MathUtil.clamp(steerAppliedVoltage_V, -12.0, 12.0));
        driveSim.update(0.02);
        steerSim.update(0.02);

        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePosition_rad = driveSim.getAngularPositionRad();
        inputs.driveVelocity_radps = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVoltage_V = driveAppliedVoltage_V;
        inputs.driveCurrent_A = Math.abs(driveSim.getCurrentDrawAmps());
        inputs.driveTemp_C = 20;

        // Update steer inputs
        inputs.steerConnected = true;
        inputs.steerPosition = new Rotation2d(steerSim.getAngularPositionRad());
        inputs.steerVelocity_radps = steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVoltage_V = steerAppliedVoltage_V;
        inputs.steerCurrent_A = Math.abs(steerSim.getCurrentDrawAmps());
        inputs.steerTemp_C = 20;

        // Update encoder inputs
        inputs.encoderConnected = true;
        inputs.encoderAbsolutePosition = new Rotation2d(steerSim.getAngularPositionRad());

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps_s = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositions_rad = new double[] {inputs.drivePosition_rad};
        inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerPosition};
    }

    @Override
    public void setDriveOpenLoop(double output_V) {
        driveClosedLoop = false;
        driveAppliedVoltage_V = output_V;
    }

    @Override
    public void setSteerOpenLoop(double output_V) {
        steerClosedLoop = false;
        steerAppliedVoltage_V = output_V;
    }

    @Override
    public void setDriveVelocity(double vel_radps) {
        driveClosedLoop = true;
        driveFFVoltage_V = DRIVE_KS * Math.signum(vel_radps) + DRIVE_KV * vel_radps;
        driveController.setSetpoint(vel_radps);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerClosedLoop = true;
        steerController.setSetpoint(rotation.getRadians());
    }
}
