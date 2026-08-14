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
import java.util.Optional;

public final class Launcher extends SubsystemBase {

    private final SubsystemWriter<Launcher> subsystemWriter = Telemetry.makeSubsystemWriter(this, "/");

    // ===Turret===
    private final Optional<SparkMax> turret;

    private final Alert
            turretDisconnectedAlert = AlertUtils.makeDisconnectAlert(TurretConfig.motorName, TurretConfig.canID),
            turretTempWarnAlert = AlertUtils.makeTempWarnAlert(TurretConfig.motorName, TurretConfig.canID),
            turretThermalShutdownAlert =
                    AlertUtils.makeThermalShutdownAlert(TurretConfig.motorName, TurretConfig.canID),
            turretBreakerAlert = AlertUtils.makeBreakerTripAlert(TurretConfig.motorName, TurretConfig.channelID);

    private TurretBuffer turretBuffer = new TurretBuffer();
    private boolean turretConnectedLast = false;
    private boolean turretThermalShutdown = false;
    private boolean turretThermalShutdownLast = false;

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
    private final Optional<SparkFlex> shooter;

    private final Alert
            shooterDisconnectedAlert = AlertUtils.makeDisconnectAlert(ShooterConfig.motorName, ShooterConfig.canID),
            shooterTempWarnAlert = AlertUtils.makeTempWarnAlert(ShooterConfig.motorName, ShooterConfig.canID),
            shooterThermalShutdownAlert =
                    AlertUtils.makeThermalShutdownAlert(ShooterConfig.motorName, ShooterConfig.canID),
            shooterBreakerAlert = AlertUtils.makeBreakerTripAlert(ShooterConfig.motorName, ShooterConfig.channelID);

    private ShooterBuffer shooterBuffer = new ShooterBuffer();
    private boolean shooterConnectedLast = false;
    private boolean shooterThermalShutdown = false;
    private boolean shooterThermalShutdownLast = false;
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
            turret = Optional.empty();

            AlertUtils.makeSystemDisabledAlert(TurretConfig.systemName).set(true);
            turretCANWriter.set(false);
            turretCANWriter.close();
        } else {
            if (Overrides.disableTurretSafety) {
                AlertUtils.makeSafetyDisabledAlert(TurretConfig.systemName).set(true);
            }

            turret = Optional.of(new SparkMax(TurretConfig.canID, MotorType.kBrushless));
            MotorUtils.safeApplyConfig(
                    turret.get(),
                    TurretConfig.motorName,
                    TurretConfig.motorConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);

            String bufferTable = getName() + "/turretBuffer";
            turretPosWriter = Telemetry.makeDoubleWriter(bufferTable, "pos", TelemetryUnits.rotations);
            turretVelWriter = Telemetry.makeDoubleWriter(bufferTable, "vel", TelemetryUnits.rpm);
            turretTempWriter = Telemetry.makeDoubleWriter(bufferTable, "temp", TelemetryUnits.celsius);
            turretAppliedOutWriter = Telemetry.makeDoubleWriter(bufferTable, "appliedOut");
            turretVoltageOutWriter = Telemetry.makeDoubleWriter(bufferTable, "voltageOut", TelemetryUnits.volts);
            turretCurrentOutWriter = Telemetry.makeDoubleWriter(bufferTable, "currentOut", TelemetryUnits.amps);
            turretConnectedWriter = Telemetry.makeBoolWriter(bufferTable, "connected");

            turretThermalShutdownWriter = Telemetry.makeBoolWriter(getName(), "turretThermalShutdown");
            turretTargetWriter = Telemetry.makeTurretAngleWriterInitialEx(getName(), "turretTarget", null, false, true);
            turretAtHomingLimitWriter = Telemetry.makeBoolWriter(getName(), "turretAtHomingLimit");
            turretBreakerWriter = Telemetry.makeBoolWriter(getName(), "turretBreakerTripped");
        }

        // ===Shooter===
        shooterCANWriter =
                Telemetry.makeBoolWriter("CAN", String.format("%s_%s", ShooterConfig.motorName, ShooterConfig.canID));
        if (Overrides.disableShooter) {
            shooter = Optional.empty();

            AlertUtils.makeSystemDisabledAlert(ShooterConfig.systemName).set(true);
            shooterCANWriter.set(false);
            shooterCANWriter.close();
        } else {
            if (Overrides.disableShooterSafety) {
                AlertUtils.makeSafetyDisabledAlert(ShooterConfig.systemName).set(true);
            }

            shooter = Optional.of(new SparkFlex(ShooterConfig.canID, MotorType.kBrushless));
            MotorUtils.safeApplyConfig(
                    shooter.get(),
                    ShooterConfig.motorName,
                    ShooterConfig.motorConfig,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);

            String bufferTable = getName() + "/shooterBuffer";
            shooterPosWriter = Telemetry.makeDoubleWriter(bufferTable, "pos", TelemetryUnits.rotations);
            shooterVelWriter = Telemetry.makeDoubleWriter(bufferTable, "vel", TelemetryUnits.rpm);
            shooterTempWriter = Telemetry.makeDoubleWriter(bufferTable, "temp", TelemetryUnits.celsius);
            shooterAppliedOutWriter = Telemetry.makeDoubleWriter(bufferTable, "appliedOut");
            shooterVoltageOutWriter = Telemetry.makeDoubleWriter(bufferTable, "voltageOut", TelemetryUnits.volts);
            shooterCurrentOutWriter = Telemetry.makeDoubleWriter(bufferTable, "currentOut", TelemetryUnits.amps);
            shooterConnectedWriter = Telemetry.makeBoolWriter(bufferTable, "connected");

            shooterThermalShutdownWriter = Telemetry.makeBoolWriter(getName(), "shooterThermalShutdown");
            shooterTargetWriter =
                    Telemetry.makeShooterTargetWriterInitialEx(getName(), "shooterTarget", null, false, true);
            shooterBreakerWriter = Telemetry.makeBoolWriter(getName(), "shooterBreakerTripped");
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
        if (turret.isEmpty()) {
            report.turretOperational = false;
        } else {
            SparkMax t = turret.get();

            turretBuffer.pos_rots = t.getEncoder().getPosition(); // frame 2
            turretBuffer.vel_RPM = t.getEncoder().getVelocity(); // frame 1
            turretBuffer.temp_C = t.getMotorTemperature(); // frame 1
            turretBuffer.appliedOut_perc = t.getAppliedOutput(); // frame 0
            turretBuffer.voltageOut_V = t.getBusVoltage() * turretBuffer.appliedOut_perc; // frame 0 & 1
            turretBuffer.currentOut_A = t.getOutputCurrent(); // frame 1
            turretBuffer.connected = !t.getFaults().can; // frame 0
            boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(TurretConfig.channelID);

            turretPosWriter.set(turretBuffer.pos_rots);
            turretVelWriter.set(turretBuffer.vel_RPM);
            turretTempWriter.set(turretBuffer.temp_C);
            turretAppliedOutWriter.set(turretBuffer.appliedOut_perc);
            turretVoltageOutWriter.set(turretBuffer.voltageOut_V);
            turretCurrentOutWriter.set(turretBuffer.currentOut_A);
            turretConnectedWriter.set(turretBuffer.connected);
            turretCANWriter.set(turretBuffer.connected);
            turretDisconnectedAlert.set(!turretBuffer.connected);
            turretAtHomingLimitWriter.set(isTurretAtHomingLimit());
            turretBreakerAlert.set(breakerTripped);
            turretBreakerWriter.set(breakerTripped);
            if (turretBuffer.connected != turretConnectedLast) {
                if (turretBuffer.connected) {
                    Telemetry.println(
                            String.format("Connected to %s (CAN %d)", TurretConfig.motorName, TurretConfig.canID));
                } else {
                    Telemetry.reportWarning(
                            String.format("Lost connection to %s (CAN %d)", TurretConfig.motorName, TurretConfig.canID),
                            false);
                }
            }

            if (Overrides.disableTurretSafety) {
                report.turretOperational = true;
            } else {
                boolean gettingToasty = turretBuffer.temp_C >= TurretConfig.tempWarnThreshold.in(Celsius);
                boolean overheating = turretBuffer.temp_C >= TurretConfig.thermalShutdownThreshold.in(Celsius);
                turretThermalShutdown = (overheating || turretThermalShutdown) && gettingToasty;

                turretThermalShutdownWriter.set(turretThermalShutdown);
                turretTempWarnAlert.set(gettingToasty && !turretThermalShutdown);
                turretThermalShutdownAlert.set(turretThermalShutdown);
                if (turretThermalShutdown != turretThermalShutdownLast) {
                    if (turretThermalShutdown) {
                        Telemetry.reportWarning(
                                String.format(
                                        "Thermal shutdown triggered on %s (CAN %d)",
                                        TurretConfig.motorName, TurretConfig.canID),
                                false);

                        stopTurret();
                    } else {
                        Telemetry.println(String.format(
                                "Thermal shutdown released on %s (CAN %d)",
                                TurretConfig.motorName, TurretConfig.canID));
                    }
                }

                report.turretOperational = turretBuffer.connected && !turretThermalShutdown && !breakerTripped;

                turretThermalShutdownLast = turretThermalShutdown;
            }

            turretConnectedLast = turretBuffer.connected;
        }
    }

    private void shooterReport(LauncherReport report) {
        if (shooter.isEmpty()) {
            report.shooterOperational = false;
            report.shooterIsAtTarget = false;
        } else {
            SparkFlex s = shooter.get();

            shooterBuffer.pos_rots = s.getEncoder().getPosition(); // frame 2
            shooterBuffer.vel_RPM = s.getEncoder().getVelocity(); // frame 1
            shooterBuffer.temp_C = s.getMotorTemperature(); // frame 1
            shooterBuffer.appliedOut_perc = s.getAppliedOutput(); // frame 0
            shooterBuffer.voltageOut_V = s.getBusVoltage() * shooterBuffer.appliedOut_perc; // frame 0 & 1
            shooterBuffer.currentOut_A = s.getOutputCurrent(); // frame 1
            shooterBuffer.connected = !s.getFaults().can; // frame 0
            boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(ShooterConfig.channelID);

            shooterPosWriter.set(shooterBuffer.pos_rots);
            shooterVelWriter.set(shooterBuffer.vel_RPM);
            shooterTempWriter.set(shooterBuffer.temp_C);
            shooterAppliedOutWriter.set(shooterBuffer.appliedOut_perc);
            shooterVoltageOutWriter.set(shooterBuffer.voltageOut_V);
            shooterCurrentOutWriter.set(shooterBuffer.currentOut_A);
            shooterConnectedWriter.set(shooterBuffer.connected);
            shooterCANWriter.set(shooterBuffer.connected);
            shooterDisconnectedAlert.set(!shooterBuffer.connected);
            shooterBreakerAlert.set(breakerTripped);
            shooterBreakerWriter.set(breakerTripped);
            if (shooterBuffer.connected != shooterConnectedLast) {
                if (shooterBuffer.connected) {
                    Telemetry.println(
                            String.format("Connected to %s (CAN %d)", ShooterConfig.motorName, ShooterConfig.canID));
                } else {
                    Telemetry.reportWarning(
                            String.format(
                                    "Lost connection to %s (CAN %d)", ShooterConfig.motorName, ShooterConfig.canID),
                            false);
                }
            }

            if (Overrides.disableShooterSafety) {
                report.shooterOperational = true;
            } else {
                boolean gettingToasty = shooterBuffer.temp_C >= ShooterConfig.tempWarnThreshold.in(Celsius);
                boolean overheating = shooterBuffer.temp_C >= ShooterConfig.thermalShutdownThreshold.in(Celsius);
                shooterThermalShutdown = (overheating || shooterThermalShutdown) && gettingToasty;

                shooterThermalShutdownWriter.set(shooterThermalShutdown);
                shooterTempWarnAlert.set(gettingToasty && !shooterThermalShutdown);
                shooterThermalShutdownAlert.set(shooterThermalShutdown);
                if (shooterThermalShutdown != shooterThermalShutdownLast) {
                    if (shooterThermalShutdown) {
                        Telemetry.reportWarning(
                                String.format(
                                        "Thermal shutdown triggered on %s (CAN %d)",
                                        ShooterConfig.motorName, ShooterConfig.canID),
                                false);

                        stopShooter();
                    } else {
                        Telemetry.println(String.format(
                                "Thermal shutdown released on %s (CAN %d)",
                                ShooterConfig.motorName, ShooterConfig.canID));
                    }
                }

                report.shooterOperational = shooterBuffer.connected && !shooterThermalShutdown && !breakerTripped;

                shooterThermalShutdownLast = shooterThermalShutdown;
            }
            report.shooterIsAtTarget = isShooterAtTarget();

            shooterConnectedLast = shooterBuffer.connected;
        }
    }

    public void setTurretAngle(TurretAngle ang) {
        if (turret.isEmpty() || turretThermalShutdown) return;

        ang.wrap();
        if (!ang.isLegal()) return;

        turret.get().getClosedLoopController().setSetpoint(ang.asMotorRotations(), ControlType.kPosition);
        turretTargetWriter.set(ang);
    }

    public void setTurretDuty(double duty) {
        if (turret.isEmpty() || (turretThermalShutdown && duty != 0)) return;

        turret.get().set(duty);
        turretTargetWriter.set(null);
    }

    public void stopTurret() {
        if (turret.isEmpty()) return;

        turret.get().stopMotor();
        turretTargetWriter.set(null);
    }

    public boolean isTurretAtHomingLimit() {
        if (turret.isEmpty()) return false;

        return turretBuffer.currentOut_A > TurretConfig.homingThresholdCurrent.in(Amps)
                || Math.abs(turretBuffer.vel_RPM) < TurretConfig.homingThresholdVel.in(RPM);
    }

    public void setAsTurretHomingPosition() {
        if (turret.isEmpty()) return;

        turret.get().getEncoder().setPosition(TurretConfig.homingEndPos.asMotorRotations());
    }

    public void setShooterVoltage(double volts) {
        if (shooter.isEmpty() || (shooterThermalShutdown && volts != 0)) return;

        shooter.get().setVoltage(volts);
        lastShooterTarget_RPM = Double.NaN;
        shooterTargetWriter.set(null);
    }

    public void setShooterTarget(ShooterTarget trg) {
        if (shooter.isEmpty() || shooterThermalShutdown) return;

        if (!trg.isLegal()) trg.clampToLegalRange();

        lastShooterTarget_RPM = trg.asShooterRPM();
        shooter.get().getClosedLoopController().setSetpoint(lastShooterTarget_RPM, ControlType.kVelocity);
        shooterTargetWriter.set(trg);
    }

    public void stopShooter() {
        if (shooter.isEmpty()) return;

        shooter.get().stopMotor();
        shooterTargetWriter.set(null);
    }

    public boolean isShooterAtTarget() {
        if (shooter.isEmpty()) return false;
        if (Double.isNaN(lastShooterTarget_RPM)) return false;

        double upwardTol = ShooterConfig.upwardTolerance.asShooterRPM();
        double downwardTol = ShooterConfig.downwardTolerance.asShooterRPM();

        return shooterBuffer.vel_RPM <= lastShooterTarget_RPM + upwardTol
                && shooterBuffer.vel_RPM >= lastShooterTarget_RPM - downwardTol;
    }
}
