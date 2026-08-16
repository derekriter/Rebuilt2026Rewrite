package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.config.LauncherConfig;
import frc.robot.config.LauncherConfig.ShooterConfig;
import frc.robot.config.LauncherConfig.TurretConfig;
import frc.robot.config.Overrides;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.telemetry.writer.compound.ShooterTargetWriter;
import frc.robot.telemetry.writer.compound.SubsystemWriter;
import frc.robot.telemetry.writer.compound.TurretAngleWriter;
import frc.robot.util.AlertUtils;
import frc.robot.util.MotorUtils;

public final class Launcher extends SubsystemBase {

    private final SubsystemWriter<Launcher> subsystemWriter =
            Telemetry.makeSubsystemWriter(this, "/", LauncherConfig.systemName);

    // ===Turret===
    private final SparkMax turret_nl;

    private final Alert turretCANAlert = AlertUtils.makeCANFailureAlert(TurretConfig.motorName),
            turretTempWarnAlert = AlertUtils.makeTempWarnAlert(TurretConfig.motorName),
            turretThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert(TurretConfig.motorName),
            turretBreakerAlert = AlertUtils.makeBreakerTripAlert(TurretConfig.motorName);

    private TurretBuffer turretBuffer = new TurretBuffer();
    private boolean turretConnectedLast = false;
    private boolean turretThermalShutdown = false;
    private boolean turretThermalShutdownLast = false;
    private boolean turretBreakerLast = false;

    private final DoubleWriter turretPosWriter;
    private final DoubleWriter turretVelWriter;
    private final DoubleWriter turretTempWriter;
    private final DoubleWriter turretAppliedOutWriter;
    private final DoubleWriter turretVoltageOutWriter;
    private final DoubleWriter turretCurrentOutWriter;
    private final BoolWriter turretConnectedWriter, turretCANWriter;
    private final BoolWriter turretThermalShutdownWriter;
    private final TurretAngleWriter turretTargetWriter;
    private final BoolWriter turretAtHomingLimitWriter;
    private final BoolWriter turretBreakerWriter;

    // ===Shooter===
    private final SparkFlex shooter_nl;

    private final Alert shooterCANAlert = AlertUtils.makeCANFailureAlert(ShooterConfig.motorName),
            shooterTempWarnAlert = AlertUtils.makeTempWarnAlert(ShooterConfig.motorName),
            shooterThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert(ShooterConfig.motorName),
            shooterBreakerAlert = AlertUtils.makeBreakerTripAlert(ShooterConfig.motorName);

    private ShooterBuffer shooterBuffer = new ShooterBuffer();
    private boolean shooterConnectedLast = false;
    private boolean shooterThermalShutdown = false;
    private boolean shooterThermalShutdownLast = false;
    private boolean shooterBreakerLast = false;
    private double lastShooterTarget_RPM = Double.NaN;

    private final DoubleWriter shooterPosWriter;
    private final DoubleWriter shooterVelWriter;
    private final DoubleWriter shooterTempWriter;
    private final DoubleWriter shooterAppliedOutWriter;
    private final DoubleWriter shooterVoltageOutWriter;
    private final DoubleWriter shooterCurrentOutWriter;
    private final BoolWriter shooterConnectedWriter, shooterCANWriter;
    private final BoolWriter shooterThermalShutdownWriter;
    private final ShooterTargetWriter shooterTargetWriter;
    private final BoolWriter shooterBreakerWriter;

    public Launcher() {
        // ===Turret===
        turretCANWriter =
                Telemetry.makeBoolWriter("CAN", String.format("%s_%s", TurretConfig.motorName, TurretConfig.canID));
        if (Overrides.disableTurret) {
            turret_nl = null;

            AlertUtils.makeSystemDisabledAlert(TurretConfig.systemName).set(true);
            turretCANWriter.set(false);
            turretCANWriter.close();
        } else {
            if (Overrides.disableTurretSafety) {
                AlertUtils.makeSafetyDisabledAlert(TurretConfig.systemName).set(true);
            }

            turret_nl = new SparkMax(TurretConfig.canID, MotorType.kBrushless);
            MotorUtils.safeApplyConfig(
                    turret_nl,
                    TurretConfig.motorName,
                    TurretConfig.canID,
                    TurretConfig.channelID,
                    TurretConfig.motorConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);

            String bufferTable = LauncherConfig.systemName + "/turretBuffer";
            turretPosWriter = Telemetry.makeDoubleWriter(bufferTable, "pos", TelemetryUnits.rotations);
            turretVelWriter = Telemetry.makeDoubleWriter(bufferTable, "vel", TelemetryUnits.rpm);
            turretTempWriter = Telemetry.makeDoubleWriter(bufferTable, "temp", TelemetryUnits.celsius);
            turretAppliedOutWriter = Telemetry.makeDoubleWriter(bufferTable, "appliedOut");
            turretVoltageOutWriter = Telemetry.makeDoubleWriter(bufferTable, "voltageOut", TelemetryUnits.volts);
            turretCurrentOutWriter = Telemetry.makeDoubleWriter(bufferTable, "currentOut", TelemetryUnits.amps);
            turretConnectedWriter = Telemetry.makeBoolWriter(bufferTable, "connected");

            turretThermalShutdownWriter = Telemetry.makeBoolWriter(LauncherConfig.systemName, "turretThermalShutdown");
            turretTargetWriter = Telemetry.makeTurretAngleWriterInitialEx(
                    LauncherConfig.systemName, "turretTarget", null, false, true);
            turretAtHomingLimitWriter = Telemetry.makeBoolWriter(LauncherConfig.systemName, "turretAtHomingLimit");
            turretBreakerWriter = Telemetry.makeBoolWriter(LauncherConfig.systemName, "turretBreakerTripped");
        }

        // ===Shooter===
        shooterCANWriter =
                Telemetry.makeBoolWriter("CAN", String.format("%s_%s", ShooterConfig.motorName, ShooterConfig.canID));
        if (Overrides.disableShooter) {
            shooter_nl = null;

            AlertUtils.makeSystemDisabledAlert(ShooterConfig.systemName).set(true);
            shooterCANWriter.set(false);
            shooterCANWriter.close();
        } else {
            if (Overrides.disableShooterSafety) {
                AlertUtils.makeSafetyDisabledAlert(ShooterConfig.systemName).set(true);
            }

            shooter_nl = new SparkFlex(ShooterConfig.canID, MotorType.kBrushless);
            MotorUtils.safeApplyConfig(
                    shooter_nl,
                    ShooterConfig.motorName,
                    ShooterConfig.canID,
                    ShooterConfig.channelID,
                    ShooterConfig.motorConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);

            String bufferTable = LauncherConfig.systemName + "/shooterBuffer";
            shooterPosWriter = Telemetry.makeDoubleWriter(bufferTable, "pos", TelemetryUnits.rotations);
            shooterVelWriter = Telemetry.makeDoubleWriter(bufferTable, "vel", TelemetryUnits.rpm);
            shooterTempWriter = Telemetry.makeDoubleWriter(bufferTable, "temp", TelemetryUnits.celsius);
            shooterAppliedOutWriter = Telemetry.makeDoubleWriter(bufferTable, "appliedOut");
            shooterVoltageOutWriter = Telemetry.makeDoubleWriter(bufferTable, "voltageOut", TelemetryUnits.volts);
            shooterCurrentOutWriter = Telemetry.makeDoubleWriter(bufferTable, "currentOut", TelemetryUnits.amps);
            shooterConnectedWriter = Telemetry.makeBoolWriter(bufferTable, "connected");

            shooterThermalShutdownWriter =
                    Telemetry.makeBoolWriter(LauncherConfig.systemName, "shooterThermalShutdown");
            shooterTargetWriter = Telemetry.makeShooterTargetWriterInitialEx(
                    LauncherConfig.systemName, "shooterTarget", null, false, true);
            shooterBreakerWriter = Telemetry.makeBoolWriter(LauncherConfig.systemName, "shooterBreakerTripped");
        }
    }

    @Override
    public void periodic() {
        subsystemWriter.update();
    }

    public void report(LauncherReport report) {
        turretReport(report);
        shooterReport(report);
    }

    private void turretReport(LauncherReport report) {
        if (turret_nl == null) {
            report.turretOperational = false;
        } else {
            turretBuffer.connected = !turret_nl.getFaults().can; // frame 0
            if (turretBuffer.connected) {
                turretBuffer.pos_rots = turret_nl.getEncoder().getPosition(); // frame 2
                turretBuffer.vel_RPM = turret_nl.getEncoder().getVelocity(); // frame 1
                turretBuffer.temp_C = turret_nl.getMotorTemperature(); // frame 1
                turretBuffer.appliedOut_perc = turret_nl.getAppliedOutput(); // frame 0
                turretBuffer.voltageOut_V = turret_nl.getBusVoltage() * turretBuffer.appliedOut_perc; // frame 0 & 1
                turretBuffer.currentOut_A = turret_nl.getOutputCurrent(); // frame 1
            } else {
                turretBuffer.pos_rots = Double.NaN;
                turretBuffer.vel_RPM = Double.NaN;
                turretBuffer.temp_C = Double.NaN;
                turretBuffer.appliedOut_perc = Double.NaN;
                turretBuffer.voltageOut_V = Double.NaN;
                turretBuffer.currentOut_A = Double.NaN;
            }
            boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(TurretConfig.channelID);

            turretConnectedWriter.set(turretBuffer.connected);
            turretCANWriter.set(turretBuffer.connected);
            turretCANAlert.set(!turretBuffer.connected && !breakerTripped);
            if (turretBuffer.connected != turretConnectedLast) {
                if (turretBuffer.connected) {
                    Telemetry.reportCANConnect(TurretConfig.motorName, TurretConfig.canID, TurretConfig.channelID);
                } else {
                    Telemetry.reportCANDisconnect(TurretConfig.motorName, TurretConfig.canID, TurretConfig.channelID);
                }
            }
            turretBreakerWriter.set(breakerTripped);
            turretBreakerAlert.set(breakerTripped);
            if (breakerTripped != turretBreakerLast) {
                if (breakerTripped) {
                    Telemetry.reportBreakerTrip(TurretConfig.motorName, TurretConfig.canID, TurretConfig.channelID);
                } else {
                    Telemetry.reportBreakerReset(TurretConfig.motorName, TurretConfig.canID, TurretConfig.channelID);
                }
            }
            turretPosWriter.set(turretBuffer.pos_rots);
            turretVelWriter.set(turretBuffer.vel_RPM);
            turretTempWriter.set(turretBuffer.temp_C);
            turretAppliedOutWriter.set(turretBuffer.appliedOut_perc);
            turretVoltageOutWriter.set(turretBuffer.voltageOut_V);
            turretCurrentOutWriter.set(turretBuffer.currentOut_A);
            turretAtHomingLimitWriter.set(isTurretAtHomingLimit());

            if (Overrides.disableTurretSafety) {
                report.turretOperational = true;
            } else {
                if (turretBuffer.connected) {
                    boolean gettingToasty = turretBuffer.temp_C >= TurretConfig.tempWarnThreshold.in(Celsius);
                    boolean overheating = turretBuffer.temp_C >= TurretConfig.thermalShutdownThreshold.in(Celsius);
                    turretThermalShutdown = (overheating || turretThermalShutdown) && gettingToasty;

                    turretTempWarnAlert.set(gettingToasty && !turretThermalShutdown);
                    turretThermalShutdownWriter.set(turretThermalShutdown);
                    turretThermalShutdownAlert.set(turretThermalShutdown);
                }

                if (turretThermalShutdown != turretThermalShutdownLast) {
                    if (turretThermalShutdown) {
                        Telemetry.reportThermalShutdownTrigger(
                                TurretConfig.motorName, TurretConfig.canID, TurretConfig.channelID);

                        stopTurret();
                    } else {
                        Telemetry.reportThermalShutdownRelease(
                                TurretConfig.motorName, TurretConfig.canID, TurretConfig.channelID);
                    }
                }

                report.turretOperational = turretBuffer.connected && !turretThermalShutdown && !breakerTripped;

                turretThermalShutdownLast = turretThermalShutdown;
            }

            turretConnectedLast = turretBuffer.connected;
            turretBreakerLast = breakerTripped;
        }
    }

    private void shooterReport(LauncherReport report) {
        if (shooter_nl == null) {
            report.shooterOperational = false;
            report.shooterIsAtTarget = false;
        } else {
            shooterBuffer.connected = !shooter_nl.getFaults().can; // frame 0
            if (shooterBuffer.connected) {
                shooterBuffer.pos_rots = shooter_nl.getEncoder().getPosition(); // frame 2
                shooterBuffer.vel_RPM = shooter_nl.getEncoder().getVelocity(); // frame 1
                shooterBuffer.temp_C = shooter_nl.getMotorTemperature(); // frame 1
                shooterBuffer.appliedOut_perc = shooter_nl.getAppliedOutput(); // frame 0
                shooterBuffer.voltageOut_V = shooter_nl.getBusVoltage() * shooterBuffer.appliedOut_perc; // frame 0 & 1
                shooterBuffer.currentOut_A = shooter_nl.getOutputCurrent(); // frame 1
            } else {
                shooterBuffer.pos_rots = Double.NaN;
                shooterBuffer.vel_RPM = Double.NaN;
                shooterBuffer.temp_C = Double.NaN;
                shooterBuffer.appliedOut_perc = Double.NaN;
                shooterBuffer.voltageOut_V = Double.NaN;
                shooterBuffer.currentOut_A = Double.NaN;
            }
            boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(ShooterConfig.channelID);

            shooterConnectedWriter.set(shooterBuffer.connected);
            shooterCANWriter.set(shooterBuffer.connected);
            shooterCANAlert.set(!shooterBuffer.connected);
            if (shooterBuffer.connected != shooterConnectedLast) {
                if (shooterBuffer.connected) {
                    Telemetry.reportCANConnect(ShooterConfig.motorName, ShooterConfig.canID, ShooterConfig.channelID);
                } else {
                    Telemetry.reportCANDisconnect(
                            ShooterConfig.motorName, ShooterConfig.canID, ShooterConfig.channelID);
                }
            }
            shooterBreakerWriter.set(breakerTripped);
            shooterBreakerAlert.set(breakerTripped);
            if (breakerTripped != shooterBreakerLast) {
                if (breakerTripped) {
                    Telemetry.reportBreakerTrip(ShooterConfig.motorName, ShooterConfig.canID, ShooterConfig.channelID);
                } else {
                    Telemetry.reportBreakerReset(ShooterConfig.motorName, ShooterConfig.canID, ShooterConfig.channelID);
                }
            }
            shooterPosWriter.set(shooterBuffer.pos_rots);
            shooterVelWriter.set(shooterBuffer.vel_RPM);
            shooterTempWriter.set(shooterBuffer.temp_C);
            shooterAppliedOutWriter.set(shooterBuffer.appliedOut_perc);
            shooterVoltageOutWriter.set(shooterBuffer.voltageOut_V);
            shooterCurrentOutWriter.set(shooterBuffer.currentOut_A);

            if (Overrides.disableShooterSafety) {
                report.shooterOperational = true;
            } else {
                if (shooterBuffer.connected) {
                    boolean gettingToasty = shooterBuffer.temp_C >= ShooterConfig.tempWarnThreshold.in(Celsius);
                    boolean overheating = shooterBuffer.temp_C >= ShooterConfig.thermalShutdownThreshold.in(Celsius);
                    shooterThermalShutdown = (overheating || shooterThermalShutdown) && gettingToasty;

                    shooterTempWarnAlert.set(gettingToasty && !shooterThermalShutdown);
                    shooterThermalShutdownWriter.set(shooterThermalShutdown);
                    shooterThermalShutdownAlert.set(shooterThermalShutdown);
                }

                if (shooterThermalShutdown != shooterThermalShutdownLast) {
                    if (shooterThermalShutdown) {
                        Telemetry.reportThermalShutdownTrigger(
                                ShooterConfig.motorName, ShooterConfig.canID, ShooterConfig.channelID);

                        stopShooter();
                    } else {
                        Telemetry.reportThermalShutdownRelease(
                                ShooterConfig.motorName, ShooterConfig.canID, ShooterConfig.channelID);
                    }
                }

                report.shooterOperational = shooterBuffer.connected && !shooterThermalShutdown && !breakerTripped;

                shooterThermalShutdownLast = shooterThermalShutdown;
            }
            report.shooterIsAtTarget = isShooterAtTarget();

            shooterConnectedLast = shooterBuffer.connected;
            shooterBreakerLast = breakerTripped;
        }
    }

    public void setTurretAngle(TurretAngle ang) {
        if (turret_nl == null || turretThermalShutdown || !turretBuffer.connected) return;

        ang.wrap();
        if (!ang.isLegal()) return;

        turret_nl.getClosedLoopController().setSetpoint(ang.asMotorRotations(), ControlType.kPosition);
        turretTargetWriter.set(ang);
    }

    public void setTurretDuty(double duty) {
        if (turret_nl == null) return;
        // allow duty to be set to 0 even when disconnected for safety reasons
        if ((turretThermalShutdown || !turretBuffer.connected) && duty != 0) return;

        turret_nl.set(duty);
        turretTargetWriter.set(null);
    }

    public void stopTurret() {
        // do not require the motor to be connected for safety reasons
        if (turret_nl == null) return;

        turret_nl.stopMotor();
        turretTargetWriter.set(null);
    }

    public boolean isTurretAtHomingLimit() {
        if (turret_nl == null || !turretBuffer.connected) return false;

        return turretBuffer.currentOut_A > TurretConfig.homingThresholdCurrent.in(Amps)
                || Math.abs(turretBuffer.vel_RPM) < TurretConfig.homingThresholdVel.in(RPM);
    }

    public void setAsTurretHomingPosition() {
        if (turret_nl == null || !turretBuffer.connected) return;

        turret_nl.getEncoder().setPosition(TurretConfig.homingEndPos.asMotorRotations());
    }

    public void setShooterVoltage(double volts) {
        if (shooter_nl == null) return;
        // allow voltage to be set to 0 even when disconnected for safety reasons
        if ((shooterThermalShutdown || !shooterBuffer.connected) && volts != 0) return;

        shooter_nl.setVoltage(volts);
        lastShooterTarget_RPM = Double.NaN;
        shooterTargetWriter.set(null);
    }

    public void setShooterTarget(ShooterTarget trg) {
        if (shooter_nl == null || shooterThermalShutdown || !shooterBuffer.connected) return;

        if (!trg.isLegal()) trg.clampToLegalRange();

        lastShooterTarget_RPM = trg.asShooterRPM();
        shooter_nl.getClosedLoopController().setSetpoint(lastShooterTarget_RPM, ControlType.kVelocity);
        shooterTargetWriter.set(trg);
    }

    public void stopShooter() {
        // do not require the motor to be connected for safety reasons
        if (shooter_nl == null) return;

        shooter_nl.stopMotor();
        shooterTargetWriter.set(null);
    }

    public boolean isShooterAtTarget() {
        if (shooter_nl == null || !shooterBuffer.connected) return false;
        if (Double.isNaN(lastShooterTarget_RPM)) return false;

        double upwardTol = ShooterConfig.upwardTolerance.asShooterRPM();
        double downwardTol = ShooterConfig.downwardTolerance.asShooterRPM();

        return shooterBuffer.vel_RPM <= lastShooterTarget_RPM + upwardTol
                && shooterBuffer.vel_RPM >= lastShooterTarget_RPM - downwardTol;
    }
}
