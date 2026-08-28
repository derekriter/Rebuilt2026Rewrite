// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.SwerveConstants;
import frc.robot.util.MotorUtils;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFX implements IModuleIO {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX steerTalon;
    private final CANcoder cancoder;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;
    private final StatusSignal<Temperature> driveTemp;

    // Inputs from steer motor
    private final StatusSignal<Angle> steerPosition;
    private final Queue<Double> steerPositionQueue;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Voltage> steerAppliedVolts;
    private final StatusSignal<Current> steerCurrent;
    private final StatusSignal<Temperature> steerTemp;

    // Inputs from encoder
    private final StatusSignal<Angle> encoderAbsolutePosition;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer steerConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer encoderConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public ModuleIOTalonFX(int index, CANBus bus) {
        constants = SwerveConstants.modules[index].constants;
        driveTalon = new TalonFX(constants.DriveMotorId, bus);
        steerTalon = new TalonFX(constants.SteerMotorId, bus);
        cancoder = new CANcoder(constants.EncoderId, bus);

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        MotorUtils.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        MotorUtils.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Slot0 = constants.SteerMotorGains;
        steerConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        steerConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            default -> throw new RuntimeException(
                    "You have selected a steer feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");};
        steerConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        steerConfig.MotionMagic.MotionMagicAcceleration = steerConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        steerConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        MotorUtils.tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerConfig, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Create timestamp queue
        timestampQueue = PhoenixOdometryThread.instance().makeTimestampQueue();

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        drivePositionQueue = PhoenixOdometryThread.instance().registerSignal(drivePosition.clone());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();
        driveTemp = driveTalon.getDeviceTemp();

        // Create steer status signals
        steerPosition = steerTalon.getPosition();
        steerPositionQueue = PhoenixOdometryThread.instance().registerSignal(steerPosition.clone());
        steerVelocity = steerTalon.getVelocity();
        steerAppliedVolts = steerTalon.getMotorVoltage();
        steerCurrent = steerTalon.getStatorCurrent();
        steerTemp = steerTalon.getDeviceTemp();

        // Create encoder status signals
        encoderAbsolutePosition = cancoder.getAbsolutePosition();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.odometryFrequency, drivePosition, steerPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                driveTemp,
                steerVelocity,
                steerAppliedVolts,
                steerCurrent,
                steerTemp,
                encoderAbsolutePosition);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus =
                BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, driveTemp);
        var steerStatus =
                BaseStatusSignal.refreshAll(steerPosition, steerVelocity, steerAppliedVolts, steerCurrent, steerTemp);
        var cancoderStatus = BaseStatusSignal.refreshAll(encoderAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePosition_rad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocity_radps = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVoltage_V = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrent_A = driveCurrent.getValueAsDouble();
        inputs.driveTemp_C = driveTemp.getValueAsDouble();

        // Update steer inputs
        inputs.steerConnected = steerConnectedDebounce.calculate(steerStatus.isOK());
        inputs.encoderConnected = encoderConnectedDebounce.calculate(cancoderStatus.isOK());
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
        inputs.steerVelocity_radps = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
        inputs.steerAppliedVoltage_V = steerAppliedVolts.getValueAsDouble();
        inputs.steerCurrent_A = steerCurrent.getValueAsDouble();
        inputs.steerTemp_C = steerTemp.getValueAsDouble();

        // Update encoder inputs
        inputs.encoderAbsolutePosition = Rotation2d.fromRotations(encoderAbsolutePosition.getValueAsDouble());

        // Update odometry inputs
        inputs.odometryTimestamps_s =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositions_rad = drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value))
                .toArray();
        inputs.odometrySteerPositions = steerPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output_V) {
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> voltageRequest.withOutput(output_V);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output_V);
                });
    }

    @Override
    public void setSteerOpenLoop(double output_V) {
        steerTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> voltageRequest.withOutput(output_V);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output_V);
                });
    }

    @Override
    public void setDriveVelocity(double vel_radps) {
        double velocityRotPerSec = Units.radiansToRotations(vel_radps);
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
                    case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
                });
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
                    case TorqueCurrentFOC -> positionTorqueCurrentRequest.withPosition(rotation.getRotations());
                });
    }
}
