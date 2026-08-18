package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.RobotContainer;
import frc.robot.config.Overrides;
import frc.robot.config.SwerveConfig;
import frc.robot.config.SwerveConfig.ModuleConfig;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.util.AlertUtils;

public class ModuleHelper {
    public final int index;
    private final ModuleConfig config;
    private final SwerveModule<TalonFX, TalonFX, CANcoder> module;

    private SwerveTalonFXBuffer driveBuffer = new SwerveTalonFXBuffer();
    private boolean driveConnectedLast = false;
    private boolean driveBreakerLast = false;
    private boolean driveThermalShutdownLast = false;
    private final StatusSignal<Temperature> driveTempSignal;
    private final StatusSignal<Double> driveAppliedOutSignal;
    private final StatusSignal<Voltage> driveVoltageOutSignal;
    private final StatusSignal<Current> driveCurrentOutSignal;
    private final StatusSignal<Current> driveCurrentInSignal;

    private final BoolWriter driveConnectedWriter;
    private final BoolWriter driveCANWriter;
    private final Alert driveCANAlert;
    private final BoolWriter driveBreakerWriter;
    private final Alert driveBreakerAlert;
    private final Alert driveTempWarnAlert;
    private final BoolWriter driveThermalShutdownWriter;
    private final Alert driveThermalShutdownAlert;
    private final DoubleWriter driveTempWriter;
    private final DoubleWriter driveAppliedOutWriter;
    private final DoubleWriter driveVoltageOutWriter;
    private final DoubleWriter driveCurrentOutWriter;
    private final DoubleWriter driveCurrentInWriter;

    private SwerveTalonFXBuffer steerBuffer = new SwerveTalonFXBuffer();
    private boolean steerConnectedLast = false;
    private boolean steerBreakerLast = false;
    private boolean steerThermalShutdownLast = false;
    private final StatusSignal<Temperature> steerTempSignal;
    private final StatusSignal<Double> steerAppliedOutSignal;
    private final StatusSignal<Voltage> steerVoltageOutSignal;
    private final StatusSignal<Current> steerCurrentOutSignal;
    private final StatusSignal<Current> steerCurrentInSignal;

    private final BoolWriter steerConnectedWriter;
    private final BoolWriter steerCANWriter;
    private final Alert steerCANAlert;
    private final BoolWriter steerBreakerWriter;
    private final Alert steerBreakerAlert;
    private final Alert steerTempWarnAlert;
    private final BoolWriter steerThermalShutdownWriter;
    private final Alert steerThermalShutdownAlert;
    private final DoubleWriter steerTempWriter;
    private final DoubleWriter steerAppliedOutWriter;
    private final DoubleWriter steerVoltageOutWriter;
    private final DoubleWriter steerCurrentOutWriter;
    private final DoubleWriter steerCurrentInWriter;

    private CANcoderBuffer encoderBuffer = new CANcoderBuffer();
    private boolean encoderConnectedLast = false;
    private boolean encoderBreakerLast = false;
    private final StatusSignal<Angle> encoderAbsPosSignal;
    private final StatusSignal<Angle> encoderPosSignal;
    private final StatusSignal<Angle> encoderPosSinceBootSignal;

    private final BoolWriter encoderConnectedWriter;
    private final BoolWriter encoderCANWriter;
    private final Alert encoderCANAlert;
    private final BoolWriter encoderBreakerWriter;
    private final Alert encoderBreakerAlert;
    private final DoubleWriter encoderAbsPosWriter;
    private final DoubleWriter encoderPosWriter;
    private final DoubleWriter encoderPosSinceBootWriter;

    public ModuleHelper(int _index, SwerveModule<TalonFX, TalonFX, CANcoder> _module) {
        index = _index;
        module = _module;
        config = SwerveConfig.modules[index];

        {
            String driveBufferTable = SwerveConfig.systemName + "/" + config.prefix + "DriveBuffer";

            TalonFX drive = module.getDriveMotor();
            driveTempSignal = drive.getDeviceTemp(false);
            driveAppliedOutSignal = drive.getDutyCycle(false);
            driveVoltageOutSignal = drive.getMotorVoltage(false);
            driveCurrentOutSignal = drive.getStatorCurrent(false);
            driveCurrentInSignal = drive.getSupplyCurrent(false);

            driveConnectedWriter = Telemetry.makeBoolWriter(driveBufferTable, "connected");
            driveCANWriter =
                    Telemetry.makeBoolWriter("CAN", String.format("%s_%s", config.driveMotorName, config.driveCANID));
            driveCANAlert = AlertUtils.makeCANFailureAlert(config.driveMotorName);
            driveBreakerWriter =
                    Telemetry.makeBoolWriter(SwerveConfig.systemName, config.prefix + "DriveBreakerTripped");
            driveBreakerAlert = AlertUtils.makeBreakerTripAlert(config.driveMotorName);
            driveTempWarnAlert = AlertUtils.makeTempWarnAlert(config.driveMotorName);
            driveThermalShutdownWriter =
                    Telemetry.makeBoolWriter(SwerveConfig.systemName, config.prefix + "DriveThermalShutdown");
            driveThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert(config.driveMotorName);
            driveTempWriter = Telemetry.makeDoubleWriter(driveBufferTable, "temp", TelemetryUnits.celsius);
            driveAppliedOutWriter = Telemetry.makeDoubleWriter(driveBufferTable, "appliedOut");
            driveVoltageOutWriter = Telemetry.makeDoubleWriter(driveBufferTable, "voltageOut", TelemetryUnits.volts);
            driveCurrentOutWriter = Telemetry.makeDoubleWriter(driveBufferTable, "currentOut", TelemetryUnits.amps);
            driveCurrentInWriter = Telemetry.makeDoubleWriter(driveBufferTable, "currentIn", TelemetryUnits.amps);
        }

        {
            String steerBufferTable = SwerveConfig.systemName + "/" + config.prefix + "SteerBuffer";

            TalonFX steer = module.getSteerMotor();
            steerTempSignal = steer.getDeviceTemp(false);
            steerAppliedOutSignal = steer.getDutyCycle(false);
            steerVoltageOutSignal = steer.getMotorVoltage(false);
            steerCurrentOutSignal = steer.getStatorCurrent(false);
            steerCurrentInSignal = steer.getSupplyCurrent(false);

            steerConnectedWriter = Telemetry.makeBoolWriter(steerBufferTable, "connected");
            steerCANWriter =
                    Telemetry.makeBoolWriter("CAN", String.format("%s_%s", config.steerMotorName, config.steerCANID));
            steerCANAlert = AlertUtils.makeCANFailureAlert(config.steerMotorName);
            steerBreakerWriter =
                    Telemetry.makeBoolWriter(SwerveConfig.systemName, config.prefix + "SteerBreakerTripped");
            steerBreakerAlert = AlertUtils.makeBreakerTripAlert(config.steerMotorName);
            steerTempWarnAlert = AlertUtils.makeTempWarnAlert(config.steerMotorName);
            steerThermalShutdownWriter =
                    Telemetry.makeBoolWriter(SwerveConfig.systemName, config.prefix + "SteerThermalShutdown");
            steerThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert(config.steerMotorName);
            steerTempWriter = Telemetry.makeDoubleWriter(steerBufferTable, "temp", TelemetryUnits.celsius);
            steerAppliedOutWriter = Telemetry.makeDoubleWriter(steerBufferTable, "appliedOut");
            steerVoltageOutWriter = Telemetry.makeDoubleWriter(steerBufferTable, "voltageOut", TelemetryUnits.volts);
            steerCurrentOutWriter = Telemetry.makeDoubleWriter(steerBufferTable, "currentOut", TelemetryUnits.amps);
            steerCurrentInWriter = Telemetry.makeDoubleWriter(steerBufferTable, "currentIn", TelemetryUnits.amps);
        }

        {
            String encoderBufferTable = SwerveConfig.systemName + "/" + config.prefix + "EncoderBuffer";

            CANcoder encoder = module.getEncoder();
            encoderAbsPosSignal = encoder.getAbsolutePosition(false);
            encoderPosSignal = encoder.getPosition(false);
            encoderPosSinceBootSignal = encoder.getPosition(false);

            encoderConnectedWriter = Telemetry.makeBoolWriter(encoderBufferTable, "connected");
            encoderCANWriter =
                    Telemetry.makeBoolWriter("CAN", String.format("%s_%s", config.encoderName, config.encoderCANID));
            encoderCANAlert = AlertUtils.makeCANFailureAlert(config.encoderName);
            encoderBreakerWriter =
                    Telemetry.makeBoolWriter(SwerveConfig.systemName, config.prefix + "SteerBreakerTripped");
            encoderBreakerAlert = AlertUtils.makeBreakerTripAlert(config.encoderName);
            encoderAbsPosWriter = Telemetry.makeDoubleWriter(encoderBufferTable, "absPos", TelemetryUnits.rotations);
            encoderPosWriter = Telemetry.makeDoubleWriter(encoderBufferTable, "pos", TelemetryUnits.rotations);
            encoderPosSinceBootWriter =
                    Telemetry.makeDoubleWriter(encoderBufferTable, "posSinceBoot", TelemetryUnits.rotations);
        }
    }

    public void report(ModuleReport report) {
        driveReport(report);
        steerReport(report);
        encoderReport(report);
    }

    private void driveReport(ModuleReport report) {
        TalonFX drive = module.getDriveMotor();

        driveBuffer.connected = drive.isConnected();
        if (driveBuffer.connected) {
            BaseStatusSignal.refreshAll(
                    driveTempSignal,
                    driveAppliedOutSignal,
                    driveVoltageOutSignal,
                    driveCurrentOutSignal,
                    driveCurrentInSignal);

            driveBuffer.temp_C = driveTempSignal.getValueAsDouble();
            driveBuffer.appliedOut_perc = driveAppliedOutSignal.getValueAsDouble();
            driveBuffer.voltageOut_V = driveVoltageOutSignal.getValueAsDouble();
            driveBuffer.currentOut_A = driveVoltageOutSignal.getValueAsDouble();
            driveBuffer.currentIn_A = driveCurrentInSignal.getValueAsDouble();
        } else {
            driveBuffer.temp_C = Double.NaN;
            driveBuffer.appliedOut_perc = Double.NaN;
            driveBuffer.voltageOut_V = Double.NaN;
            driveBuffer.currentOut_A = Double.NaN;
            driveBuffer.currentIn_A = Double.NaN;
        }
        boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(config.driveChannelID);

        driveConnectedWriter.set(driveBuffer.connected);
        driveCANWriter.set(driveBuffer.connected);
        driveCANAlert.set(!driveBuffer.connected && !breakerTripped);
        if (driveBuffer.connected != driveConnectedLast) {
            if (driveBuffer.connected) {
                Telemetry.reportCANConnect(config.driveMotorName, config.driveCANID, config.driveChannelID);
            } else {
                Telemetry.reportCANDisconnect(config.driveMotorName, config.driveCANID, config.driveChannelID);
            }
        }
        driveBreakerWriter.set(breakerTripped);
        driveBreakerAlert.set(breakerTripped);
        if (breakerTripped != driveBreakerLast) {
            if (breakerTripped) {
                Telemetry.reportBreakerTrip(config.driveMotorName, config.driveCANID, config.driveChannelID);
            } else {
                Telemetry.reportBreakerReset(config.driveMotorName, config.driveCANID, config.driveChannelID);
            }
        }
        driveTempWriter.set(driveBuffer.temp_C);
        driveAppliedOutWriter.set(driveBuffer.appliedOut_perc);
        driveVoltageOutWriter.set(driveBuffer.voltageOut_V);
        driveCurrentOutWriter.set(driveBuffer.currentOut_A);
        driveCurrentInWriter.set(driveBuffer.currentIn_A);

        if (Overrides.disableSwerveSafety) {
            report.driveOperational = true;
            report.driveThermalShutdown = false;
        } else {
            if (driveBuffer.connected) {
                boolean gettingToasty = driveBuffer.temp_C >= SwerveConfig.driveTempWarnThreshold.in(Celsius);
                boolean overheating = driveBuffer.temp_C >= SwerveConfig.driveThermalShutdownThreshold.in(Celsius);
                report.driveThermalShutdown = (overheating || report.driveThermalShutdown) && gettingToasty;

                driveTempWarnAlert.set(gettingToasty && !report.driveThermalShutdown);
                driveThermalShutdownWriter.set(report.driveThermalShutdown);
                driveThermalShutdownAlert.set(report.driveThermalShutdown);
            }

            if (report.driveThermalShutdown != driveThermalShutdownLast) {
                if (report.driveThermalShutdown) {
                    Telemetry.reportThermalShutdownTrigger(
                            config.driveMotorName, config.driveCANID, config.driveChannelID);
                } else {
                    Telemetry.reportThermalShutdownRelease(
                            config.driveMotorName, config.driveCANID, config.driveChannelID);
                }
            }

            report.driveOperational = driveBuffer.connected && !report.driveThermalShutdown && !breakerTripped;

            driveThermalShutdownLast = report.driveThermalShutdown;
        }

        driveConnectedLast = driveBuffer.connected;
        driveBreakerLast = breakerTripped;
    }

    private void steerReport(ModuleReport report) {
        TalonFX steer = module.getSteerMotor();

        steerBuffer.connected = steer.isConnected();
        if (steerBuffer.connected) {
            BaseStatusSignal.refreshAll(
                    steerTempSignal,
                    steerAppliedOutSignal,
                    steerVoltageOutSignal,
                    steerCurrentOutSignal,
                    steerCurrentInSignal);

            steerBuffer.temp_C = steerTempSignal.getValueAsDouble();
            steerBuffer.appliedOut_perc = steerAppliedOutSignal.getValueAsDouble();
            steerBuffer.voltageOut_V = steerVoltageOutSignal.getValueAsDouble();
            steerBuffer.currentOut_A = steerVoltageOutSignal.getValueAsDouble();
            steerBuffer.currentIn_A = steerCurrentInSignal.getValueAsDouble();
        } else {
            steerBuffer.temp_C = Double.NaN;
            steerBuffer.appliedOut_perc = Double.NaN;
            steerBuffer.voltageOut_V = Double.NaN;
            steerBuffer.currentOut_A = Double.NaN;
            steerBuffer.currentIn_A = Double.NaN;
        }
        boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(config.steerChannelID);

        steerConnectedWriter.set(steerBuffer.connected);
        steerCANWriter.set(steerBuffer.connected);
        steerCANAlert.set(!steerBuffer.connected && !breakerTripped);
        if (steerBuffer.connected != steerConnectedLast) {
            if (steerBuffer.connected) {
                Telemetry.reportCANConnect(config.steerMotorName, config.steerCANID, config.steerChannelID);
            } else {
                Telemetry.reportCANDisconnect(config.steerMotorName, config.steerCANID, config.steerChannelID);
            }
        }
        steerBreakerWriter.set(breakerTripped);
        steerBreakerAlert.set(breakerTripped);
        if (breakerTripped != steerBreakerLast) {
            if (breakerTripped) {
                Telemetry.reportBreakerTrip(config.steerMotorName, config.steerCANID, config.steerChannelID);
            } else {
                Telemetry.reportBreakerReset(config.steerMotorName, config.steerCANID, config.steerChannelID);
            }
        }
        steerTempWriter.set(steerBuffer.temp_C);
        steerAppliedOutWriter.set(steerBuffer.appliedOut_perc);
        steerVoltageOutWriter.set(steerBuffer.voltageOut_V);
        steerCurrentOutWriter.set(steerBuffer.currentOut_A);
        steerCurrentInWriter.set(steerBuffer.currentIn_A);

        if (Overrides.disableSwerveSafety) {
            report.steerOperational = true;
            report.steerThermalShutdown = false;
        } else {
            if (steerBuffer.connected) {
                boolean gettingToasty = steerBuffer.temp_C >= SwerveConfig.steerTempWarnThreshold.in(Celsius);
                boolean overheating = steerBuffer.temp_C >= SwerveConfig.steerThermalShutdownThreshold.in(Celsius);
                report.steerThermalShutdown = (overheating || report.steerThermalShutdown) && gettingToasty;

                steerTempWarnAlert.set(gettingToasty && !report.steerThermalShutdown);
                steerThermalShutdownWriter.set(report.steerThermalShutdown);
                steerThermalShutdownAlert.set(report.steerThermalShutdown);
            }

            if (report.steerThermalShutdown != steerThermalShutdownLast) {
                if (report.steerThermalShutdown) {
                    Telemetry.reportThermalShutdownTrigger(
                            config.steerMotorName, config.steerCANID, config.steerChannelID);
                } else {
                    Telemetry.reportThermalShutdownRelease(
                            config.steerMotorName, config.steerCANID, config.steerChannelID);
                }
            }

            report.steerOperational = steerBuffer.connected && !report.steerThermalShutdown && !breakerTripped;

            steerThermalShutdownLast = report.steerThermalShutdown;
        }

        steerConnectedLast = steerBuffer.connected;
        steerBreakerLast = breakerTripped;
    }

    private void encoderReport(ModuleReport report) {
        CANcoder encoder = module.getEncoder();

        encoderBuffer.connected = encoder.isConnected();
        if (encoderBuffer.connected) {
            BaseStatusSignal.refreshAll(encoderAbsPosSignal, encoderPosSignal, encoderPosSinceBootSignal);

            encoderBuffer.absPos_rots = encoderAbsPosSignal.getValueAsDouble();
            encoderBuffer.pos_rots = encoderPosSignal.getValueAsDouble();
            encoderBuffer.posSinceBoot_rots = encoderPosSinceBootSignal.getValueAsDouble();
        } else {
            encoderBuffer.absPos_rots = Double.NaN;
            encoderBuffer.pos_rots = Double.NaN;
            encoderBuffer.posSinceBoot_rots = Double.NaN;
        }
        boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(config.encoderChannelID);

        encoderConnectedWriter.set(encoderBuffer.connected);
        encoderCANWriter.set(encoderBuffer.connected);
        encoderCANAlert.set(!encoderBuffer.connected && !breakerTripped);
        if (encoderBuffer.connected != encoderConnectedLast) {
            if (encoderBuffer.connected) {
                Telemetry.reportCANConnect(config.encoderName, config.encoderCANID, config.encoderChannelID);
            } else {
                Telemetry.reportCANDisconnect(config.encoderName, config.encoderCANID, config.encoderChannelID);
            }
        }
        encoderBreakerWriter.set(breakerTripped);
        encoderBreakerAlert.set(breakerTripped);
        if (breakerTripped != encoderBreakerLast) {
            if (breakerTripped) {
                Telemetry.reportBreakerTrip(config.encoderName, config.driveCANID, config.driveChannelID);
            } else {
                Telemetry.reportBreakerReset(config.encoderName, config.encoderCANID, config.driveChannelID);
            }
        }
        encoderAbsPosWriter.set(encoderBuffer.absPos_rots);
        encoderPosWriter.set(encoderBuffer.pos_rots);
        encoderPosSinceBootWriter.set(encoderBuffer.posSinceBoot_rots);

        if (Overrides.disableSwerveSafety) {
            report.encoderOperational = true;
        } else {
            report.encoderOperational = encoderBuffer.connected && !breakerTripped;
        }

        encoderConnectedLast = encoderBuffer.connected;
        encoderBreakerLast = breakerTripped;
    }
}
