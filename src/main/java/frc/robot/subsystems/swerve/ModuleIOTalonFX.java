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
    private final StatusSignal<Angle> drivePosition_rots;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity_rps;
    private final StatusSignal<Temperature> driveTemp_C;
    private final StatusSignal<Voltage> driveAppliedVoltage_V;
    private final StatusSignal<Current> driveStatorCurrent_A;
    private final StatusSignal<Current> driveSupplyCurrent_A;

    // Inputs from steer motor
    private final StatusSignal<Angle> steerPosition_rots;
    private final Queue<Double> steerPositionQueue;
    private final StatusSignal<AngularVelocity> steerVelocity_rps;
    private final StatusSignal<Temperature> steerTemp_C;
    private final StatusSignal<Voltage> steerAppliedVoltage_V;
    private final StatusSignal<Current> steerStatorCurrent_A;
    private final StatusSignal<Current> steerSupplyCurrent_A;

    // Inputs from encoder
    private final StatusSignal<Angle> encoderAbsolutePosition_rots;

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
        drivePosition_rots = driveTalon.getPosition();
        drivePositionQueue = PhoenixOdometryThread.instance().registerSignal(drivePosition_rots.clone());
        driveVelocity_rps = driveTalon.getVelocity();
        driveTemp_C = driveTalon.getDeviceTemp();
        driveAppliedVoltage_V = driveTalon.getMotorVoltage();
        driveStatorCurrent_A = driveTalon.getStatorCurrent();
        driveSupplyCurrent_A = driveTalon.getSupplyCurrent();

        // Create steer status signals
        steerPosition_rots = steerTalon.getPosition();
        steerPositionQueue = PhoenixOdometryThread.instance().registerSignal(steerPosition_rots.clone());
        steerVelocity_rps = steerTalon.getVelocity();
        steerTemp_C = steerTalon.getDeviceTemp();
        steerAppliedVoltage_V = steerTalon.getMotorVoltage();
        steerStatorCurrent_A = steerTalon.getStatorCurrent();
        steerSupplyCurrent_A = steerTalon.getSupplyCurrent();

        // Create encoder status signals
        encoderAbsolutePosition_rots = cancoder.getAbsolutePosition();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
                SwerveConstants.odometryFrequency, drivePosition_rots, steerPosition_rots);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity_rps,
                driveTemp_C,
                driveAppliedVoltage_V,
                driveStatorCurrent_A,
                driveSupplyCurrent_A,
                steerVelocity_rps,
                steerTemp_C,
                steerAppliedVoltage_V,
                steerStatorCurrent_A,
                steerSupplyCurrent_A,
                encoderAbsolutePosition_rots);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, steerTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(
                drivePosition_rots,
                driveVelocity_rps,
                driveTemp_C,
                driveAppliedVoltage_V,
                driveStatorCurrent_A,
                driveSupplyCurrent_A);
        var steerStatus = BaseStatusSignal.refreshAll(
                steerPosition_rots,
                steerVelocity_rps,
                steerTemp_C,
                steerAppliedVoltage_V,
                steerStatorCurrent_A,
                steerSupplyCurrent_A);
        var cancoderStatus = BaseStatusSignal.refreshAll(encoderAbsolutePosition_rots);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePosition_rad = Units.rotationsToRadians(drivePosition_rots.getValueAsDouble());
        inputs.driveVelocity_radps = Units.rotationsToRadians(driveVelocity_rps.getValueAsDouble());
        inputs.driveTemp_C = driveTemp_C.getValueAsDouble();
        inputs.driveAppliedVoltage_V = driveAppliedVoltage_V.getValueAsDouble();
        inputs.driveStatorCurrent_A = driveStatorCurrent_A.getValueAsDouble();

        // Update steer inputs
        inputs.steerConnected = steerConnectedDebounce.calculate(steerStatus.isOK());
        inputs.encoderConnected = encoderConnectedDebounce.calculate(cancoderStatus.isOK());
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition_rots.getValueAsDouble());
        inputs.steerVelocity_radps = Units.rotationsToRadians(steerVelocity_rps.getValueAsDouble());
        inputs.steerTemp_C = steerTemp_C.getValueAsDouble();
        inputs.steerAppliedVoltage_V = steerAppliedVoltage_V.getValueAsDouble();
        inputs.steerStatorCurrent_A = steerStatorCurrent_A.getValueAsDouble();

        // Update encoder inputs
        inputs.encoderAbsolutePosition = Rotation2d.fromRotations(encoderAbsolutePosition_rots.getValueAsDouble());

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
