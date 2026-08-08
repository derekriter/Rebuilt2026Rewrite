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
import frc.robot.config.LauncherConfig.ShooterConfig;
import frc.robot.config.LauncherConfig.TurretConfig;
import frc.robot.config.Overrides;
import frc.robot.telemetry.Telemetry;
import frc.robot.util.AlertUtils;
import frc.robot.util.MotorUtils;
import java.util.Optional;

public final class Launcher extends SubsystemBase {

    // ===Turret===
    private Optional<SparkMax> turret;

    private final Alert
            turretDisconnectedAlert = AlertUtils.makeDisconnectAlert(TurretConfig.motorName, TurretConfig.canID),
            turretTempWarnAlert = AlertUtils.makeTempWarnAlert(TurretConfig.motorName, TurretConfig.canID),
            turretThermalShutdownAlert =
                    AlertUtils.makeThermalShutdownAlert(TurretConfig.motorName, TurretConfig.canID);

    private boolean turretConnectedLast = false;
    private boolean turretThermalShutdown = false;
    private boolean turretThermalShutdownLast = false;

    // ===Shooter===
    private Optional<SparkFlex> shooter;

    private final Alert
            shooterDisconnectedAlert = AlertUtils.makeDisconnectAlert(ShooterConfig.motorName, ShooterConfig.canID),
            shooterTempWarnAlert = AlertUtils.makeTempWarnAlert(ShooterConfig.motorName, ShooterConfig.canID),
            shooterThermalShutdownAlert =
                    AlertUtils.makeThermalShutdownAlert(ShooterConfig.motorName, ShooterConfig.canID);

    private boolean shooterConnectedLast = false;
    private boolean shooterThermalShutdown = false;
    private boolean shooterThermalShutdownLast = false;
    private double lastShooterTarget = Double.NaN;

    public Launcher() {
        // ===Turret===
        if (Overrides.disableTurret) {
            turret = Optional.empty();

            AlertUtils.makeSystemDisabledAlert(TurretConfig.systemName).set(true);
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
        }

        // ===Shooter===
        if (Overrides.disableShooter) {
            shooter = Optional.empty();

            AlertUtils.makeSystemDisabledAlert(ShooterConfig.systemName).set(true);
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
        }
    }

    @Override
    public void periodic() {}

    public void pollFlags(LauncherFlags flags) {
        if (turret.isEmpty()) {
            flags.turretOperational = false;
        } else {
            SparkMax t = turret.get();
            boolean connected = !t.getFaults().can;

            if (Overrides.disableTurretSafety) {
                flags.turretOperational = true;
            } else {
                double temp = t.getMotorTemperature();

                boolean gettingToasty = temp >= TurretConfig.tempWarnThreshold.in(Celsius);
                boolean overheating = temp >= TurretConfig.thermalShutdownThreshold.in(Celsius);
                turretThermalShutdown = (overheating || turretThermalShutdown) && gettingToasty;

                flags.turretOperational = connected && !turretThermalShutdown;

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

                turretThermalShutdownLast = turretThermalShutdown;
            }

            turretDisconnectedAlert.set(!connected);

            if (connected != turretConnectedLast) {
                if (connected) {
                    Telemetry.println(
                            String.format("Connected to %s (CAN %d)", TurretConfig.motorName, TurretConfig.canID));
                } else {
                    Telemetry.reportWarning(
                            String.format("Lost connection to %s (CAN %d)", TurretConfig.motorName, TurretConfig.canID),
                            false);
                }
            }

            turretConnectedLast = connected;
        }

        if (shooter.isEmpty()) {
            flags.shooterOperational = false;
        } else {
            SparkFlex s = shooter.get();
            boolean connected = !s.getFaults().can;

            if (Overrides.disableShooterSafety) {
                flags.shooterOperational = true;
            } else {
                double temp = s.getMotorTemperature();

                boolean gettingToasty = temp >= ShooterConfig.tempWarnThreshold.in(Celsius);
                boolean overheating = temp >= ShooterConfig.thermalShutdownThreshold.in(Celsius);
                shooterThermalShutdown = (overheating || shooterThermalShutdown) && gettingToasty;

                flags.shooterOperational = connected && !shooterThermalShutdown;

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

                shooterThermalShutdownLast = shooterThermalShutdown;
            }
            flags.shooterIsAtTarget = isShooterAtTarget();

            shooterDisconnectedAlert.set(!connected);

            if (connected != shooterConnectedLast) {
                if (connected) {
                    Telemetry.println(
                            String.format("Connected to %s (CAN %d)", ShooterConfig.motorName, ShooterConfig.canID));
                } else {
                    Telemetry.reportWarning(
                            String.format(
                                    "Lost connection to %s (CAN %d)", ShooterConfig.motorName, ShooterConfig.canID),
                            false);
                }
            }

            shooterConnectedLast = connected;
        }
    }

    public void setTurretAngle(TurretAngle ang) {
        if (turret.isEmpty() || turretThermalShutdown) return;

        ang.wrap();
        if (!ang.isLegal()) return;

        turret.get().getClosedLoopController().setSetpoint(ang.asMotorRotations(), ControlType.kPosition);
    }

    public void setTurretDuty(double duty) {
        if (turret.isEmpty() || (turretThermalShutdown && duty != 0)) return;

        turret.get().set(duty);
    }

    public void stopTurret() {
        if (turret.isEmpty()) return;

        turret.get().stopMotor();
    }

    public boolean isTurretAtHomingLimit() {
        if (turret.isEmpty()) return false;

        SparkMax t = turret.get();
        return t.getOutputCurrent() > TurretConfig.homingThresholdCurrent.in(Amps)
                || Math.abs(t.getEncoder().getVelocity()) < TurretConfig.homingThresholdVel.in(RPM);
    }

    public void setAsTurretHomingPosition() {
        if (turret.isEmpty()) return;

        turret.get().getEncoder().setPosition(TurretConfig.homingEndPos.asMotorRotations());
    }

    public void setShooterVoltage(double volts) {
        if (shooter.isEmpty() || (shooterThermalShutdown && volts != 0)) return;

        shooter.get().setVoltage(volts);
        lastShooterTarget = Double.NaN;
    }

    public void setShooterTarget(ShooterTarget trg) {
        if (shooter.isEmpty() || shooterThermalShutdown) return;

        if (!trg.isLegal()) trg.clampToLegalRange();

        lastShooterTarget = trg.asShooterRPM();
        shooter.get().getClosedLoopController().setSetpoint(lastShooterTarget, ControlType.kVelocity);
    }

    public void stopShooter() {
        if (shooter.isEmpty()) return;

        shooter.get().stopMotor();
    }

    public boolean isShooterAtTarget() {
        if (shooter.isEmpty()) return false;
        if (Double.isNaN(lastShooterTarget)) return false;

        double vel = shooter.get().getEncoder().getVelocity();
        double upwardTol = ShooterConfig.upwardTolerance.asShooterRPM();
        double downwardTol = ShooterConfig.downwardTolerance.asShooterRPM();

        return vel <= lastShooterTarget + upwardTol && vel >= lastShooterTarget - downwardTol;
    }
}
