package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.ShooterConstants;
import frc.robot.constants.LauncherConstants.TurretConstants;
import frc.robot.constants.Overrides;
import frc.robot.subsystems.launcher.shooter.IShooterIO;
import frc.robot.subsystems.launcher.shooter.IShooterIO.ShooterIOInputs;
import frc.robot.subsystems.launcher.shooter.ShooterTarget;
import frc.robot.subsystems.launcher.turret.ITurretIO;
import frc.robot.subsystems.launcher.turret.ITurretIO.TurretIOInputs;
import frc.robot.subsystems.launcher.turret.TurretAngle;
import frc.robot.util.AlertUtils;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;

public final class Launcher extends SubsystemBase {

    // ===Turret===
    private final ITurretIO turretIO_nl;

    private final Alert turretCANAlert = AlertUtils.makeCANFailureAlert("turret"),
            turretBreakerAlert = AlertUtils.makeBreakerTripAlert("turret");

    private TurretIOInputs turretInputs = new TurretIOInputs();
    private boolean turretConnectedLast = false;
    private boolean turretBreaker = false;
    private boolean turretBreakerLast = false;
    private double lastTurretTarget_rot = Double.NaN;

    // ===Shooter===
    private final IShooterIO shooterIO_nl;

    private final Alert shooterCANAlert = AlertUtils.makeCANFailureAlert("shooter"),
            shooterTempWarnAlert = AlertUtils.makeTempWarnAlert("shooter"),
            shooterThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert("shooter"),
            shooterBreakerAlert = AlertUtils.makeBreakerTripAlert("shooter");

    private ShooterIOInputs shooterInputs = new ShooterIOInputs();
    private boolean shooterConnectedLast = false;
    private boolean shooterThermalShutdown = false;
    private boolean shooterThermalShutdownLast = false;
    private boolean shooterBreaker = false;
    private boolean shooterBreakerLast = false;
    private double lastShooterTarget_RPM = Double.NaN;

    public Launcher(ITurretIO _turretIO_nl, IShooterIO _shooterIO_nl) {
        // ===Turret===
        turretIO_nl = _turretIO_nl;
        if (turretIO_nl == null) {
            Logger.recordOutput("CAN/turret_" + TurretConstants.canID, false);
            AlertUtils.makeSystemDisabledAlert("turret").set(true);
        } else {
            if (Overrides.disableTurretSafety) {
                AlertUtils.makeSafetyDisabledAlert("turret").set(true);
            }
        }

        // ===Shooter===
        shooterIO_nl = _shooterIO_nl;
        if (shooterIO_nl == null) {
            Logger.recordOutput("CAN/shooter_" + ShooterConstants.canID, false);
            AlertUtils.makeSystemDisabledAlert("shooter").set(true);
        } else {
            if (Overrides.disableShooterSafety) {
                AlertUtils.makeSafetyDisabledAlert("shooter").set(true);
            }
        }
    }

    @Override
    public void periodic() {
        if (turretIO_nl != null) {
            turretIO_nl.updateInputs(turretInputs);
            Logger.processInputs("LauncherInputs/TurretInputs", turretInputs);

            turretBreaker = RobotContainer.instance().pdh.isBreakerTripped(TurretConstants.channelID);

            Logger.recordOutput("CAN/turret_" + TurretConstants.canID, turretInputs.connected);
            turretCANAlert.set(!turretInputs.connected && !turretBreaker);
            if (turretInputs.connected != turretConnectedLast) {
                if (turretInputs.connected) {
                    Console.reportCANConnect("turret", TurretConstants.canID, TurretConstants.channelID);
                } else {
                    Console.reportCANDisconnect("turret", TurretConstants.canID, TurretConstants.channelID);
                }
            }
            Logger.recordOutput("Launcher/Turret/breakerTripped", turretBreaker);
            turretBreakerAlert.set(turretBreaker);
            if (turretBreaker != turretBreakerLast) {
                if (turretBreaker) {
                    Console.reportBreakerTrip("turret", TurretConstants.canID, TurretConstants.channelID);
                } else {
                    Console.reportBreakerReset("turret", TurretConstants.canID, TurretConstants.channelID);
                }
            }
            Logger.recordOutput("Launcher/Turret/atHomingLimit", isTurretAtHomingLimit());

            turretConnectedLast = turretInputs.connected;
            turretBreakerLast = turretBreaker;
        }

        if (shooterIO_nl != null) {
            shooterIO_nl.updateInputs(shooterInputs);
            Logger.processInputs("LauncherInputs/ShooterInputs", shooterInputs);

            shooterBreaker = RobotContainer.instance().pdh.isBreakerTripped(ShooterConstants.channelID);

            Logger.recordOutput("CAN/shooter_" + ShooterConstants.canID, shooterInputs.connected);
            shooterCANAlert.set(!shooterInputs.connected && !shooterBreaker);
            if (shooterInputs.connected != shooterConnectedLast) {
                if (shooterInputs.connected) {
                    Console.reportCANConnect("shooter", ShooterConstants.canID, ShooterConstants.channelID);
                } else {
                    Console.reportCANDisconnect("shooter", ShooterConstants.canID, ShooterConstants.channelID);
                }
            }
            Logger.recordOutput("Launcher/Shooter/breakerTripped", shooterBreaker);
            shooterBreakerAlert.set(shooterBreaker);
            if (shooterBreaker != shooterBreakerLast) {
                if (shooterBreaker) {
                    Console.reportBreakerTrip("shooter", ShooterConstants.canID, ShooterConstants.channelID);
                } else {
                    Console.reportBreakerReset("shooter", ShooterConstants.canID, ShooterConstants.channelID);
                }
            }

            if (!Overrides.disableShooterSafety && shooterInputs.connected) {
                boolean gettingToasty = shooterInputs.temp_C >= ShooterConstants.tempWarnThreshold.in(Celsius);
                boolean overheating = shooterInputs.temp_C >= ShooterConstants.thermalShutdownThreshold.in(Celsius);
                shooterThermalShutdown = (overheating || shooterThermalShutdown) && gettingToasty;

                shooterTempWarnAlert.set(gettingToasty && !shooterThermalShutdown);
                Logger.recordOutput("Launcher/Shooter/thermalShutdown", shooterThermalShutdown);
                shooterThermalShutdownAlert.set(shooterThermalShutdown);
            }

            if (shooterThermalShutdown != shooterThermalShutdownLast) {
                if (shooterThermalShutdown) {
                    Console.reportThermalShutdownTrigger("shooter", ShooterConstants.canID, ShooterConstants.channelID);

                    stopShooter();
                } else {
                    Console.reportThermalShutdownRelease("shooter", ShooterConstants.canID, ShooterConstants.channelID);
                }
            }

            shooterConnectedLast = shooterInputs.connected;
            shooterBreakerLast = shooterBreaker;
            shooterThermalShutdownLast = shooterThermalShutdown;
        }

        Command currentCommand = getCurrentCommand();
        Logger.recordOutput("Launcher/currentCommand", currentCommand == null ? null : currentCommand.getName());

        Command defaultCommand = getDefaultCommand();
        Logger.recordOutput("Launcher/defaultCommand", defaultCommand == null ? null : defaultCommand.getName());

        Translation2d launcherTranslation = getLauncherTranslation();
        Rotation2d robotRotation = RobotContainer.instance().swerve.getRotation();

        Rotation2d turretRealRotation;
        Rotation2d turretTargetRotation;
        if (turretIO_nl != null) {
            turretRealRotation = robotRotation.plus(
                    new Rotation2d(TurretAngle.motorAngleToMechAngle_ul(turretInputs.pos_rots * 2 * Math.PI)));
            Logger.recordOutput("Launcher/Turret/pose", new Pose2d(launcherTranslation, turretRealRotation));

            if (Double.isNaN(lastTurretTarget_rot)) {
                Logger.recordOutput("Launcher/Turret/targetPose", BlankValues.pose2d);
                turretTargetRotation = turretRealRotation;
            } else {
                turretTargetRotation = robotRotation.plus(
                        new Rotation2d(TurretAngle.motorAngleToMechAngle_ul(lastTurretTarget_rot * 2 * Math.PI)));
                Logger.recordOutput(
                        "Launcher/Turret/targetPose", new Pose2d(launcherTranslation, turretTargetRotation));
            }
        } else {
            turretRealRotation = robotRotation;
            turretTargetRotation = robotRotation;
        }
        if (shooterIO_nl != null) {
            Logger.recordOutput(
                    "Launcher/Shooter/point",
                    launcherTranslation.plus(new Translation2d(ShooterTarget.velToDistance_m(shooterInputs.vel_RPM), 0)
                            .rotateBy(turretRealRotation)));

            if (Double.isNaN(lastShooterTarget_RPM)) {
                Logger.recordOutput("Launcher/Shooter/targetPoint", BlankValues.translation2d);
            } else {
                Logger.recordOutput(
                        "Launcher/Shooter/targetPoint",
                        launcherTranslation.plus(
                                new Translation2d(ShooterTarget.velToDistance_m(lastShooterTarget_RPM), 0)
                                        .rotateBy(turretTargetRotation)));
            }
        }
    }

    public void report(LauncherReport report) {
        if (turretIO_nl == null) {
            report.turretOperational = false;
            report.turretIsAtTarget = false;
        } else {
            if (Overrides.disableTurretSafety) {
                report.turretOperational = true;
            } else {
                report.turretOperational = turretInputs.connected && !turretBreaker;
            }

            report.turretIsAtTarget = isTurretAtTarget();
        }

        if (shooterIO_nl == null) {
            report.shooterOperational = false;
            report.shooterIsAtTarget = false;
        } else {
            if (Overrides.disableShooterSafety) {
                report.shooterOperational = true;
            } else {
                report.shooterOperational = shooterInputs.connected && !shooterThermalShutdown && !shooterBreaker;
            }

            report.shooterIsAtTarget = isShooterAtTarget();
        }
    }

    private Translation2d getLauncherTranslation() {
        Pose2d robotPose = RobotContainer.instance().swerve.getPose();
        return robotPose.getTranslation().plus(LauncherConstants.launcherOffset.rotateBy(robotPose.getRotation()));
    }

    public void setTurretAngle(TurretAngle ang) {
        if (turretIO_nl == null) return;

        ang.wrap();
        if (!ang.isLegal()) return;

        lastTurretTarget_rot = ang.asMotorRotations();
        turretIO_nl.setPositionTarget(lastTurretTarget_rot);

        Logger.recordOutput("Launcher/Turret/motorTarget", lastTurretTarget_rot, Rotations);
        Logger.recordOutput("Launcher/Turret/mechTarget", ang.asMechanismDegrees(), Degrees);
    }

    public void setTurretVoltage(double voltage_volts) {
        if (turretIO_nl == null) return;

        turretIO_nl.setVoltage(voltage_volts);
        lastTurretTarget_rot = Double.NaN;

        Logger.recordOutput("Launcher/Turret/motorTarget", Double.NaN, Rotations);
        Logger.recordOutput("Launcher/Turret/mechTarget", Double.NaN, Degrees);
    }

    public void stopTurret() {
        if (turretIO_nl == null) return;

        turretIO_nl.stop();
        lastTurretTarget_rot = Double.NaN;

        Logger.recordOutput("Launcher/Turret/motorTarget", Double.NaN, Rotations);
        Logger.recordOutput("Launcher/Turret/mechTarget", Double.NaN, Degrees);
    }

    public boolean isTurretAtHomingLimit() {
        if (turretIO_nl == null || !turretInputs.connected) return false;

        return turretInputs.currentOut_A > TurretConstants.homingThresholdCurrent.in(Amps)
                || Math.abs(turretInputs.vel_RPM) < TurretConstants.homingThresholdVel.in(RPM);
    }

    public void setAsTurretHomingPosition() {
        if (turretIO_nl == null) return;

        turretIO_nl.setEncoderPosition(TurretConstants.homingEndPos.asMotorRotations());
    }

    public boolean isTurretAtTarget() {
        if (turretIO_nl == null || !turretInputs.connected) return false;
        if (Double.isNaN(lastTurretTarget_rot)) return false;

        return MathUtil.isNear(
                lastTurretTarget_rot, turretInputs.pos_rots, TurretConstants.targetTolerance.asMotorRotations());
    }

    public void setShooterTarget(ShooterTarget trg) {
        if (shooterIO_nl == null || shooterThermalShutdown) return;

        if (!trg.isLegal()) trg.clampToLegalRange();

        lastShooterTarget_RPM = trg.asShooterRPM();
        shooterIO_nl.setVelocityTarget(lastShooterTarget_RPM);

        Logger.recordOutput("Launcher/Shooter/velTarget", lastShooterTarget_RPM, RPM);
        Logger.recordOutput("Launcher/Shooter/distTarget", trg.asMetersToTarget(), Meters);
    }

    public void setShooterVoltage(double volts) {
        if (shooterIO_nl == null) return;
        if (shooterThermalShutdown && volts != 0) return;

        shooterIO_nl.setVoltage(volts);
        lastShooterTarget_RPM = Double.NaN;

        Logger.recordOutput("Launcher/Shooter/velTarget", Double.NaN, RPM);
        Logger.recordOutput("Launcher/Shooter/distTarget", Double.NaN, Meters);
    }

    public void stopShooter() {
        if (shooterIO_nl == null) return;

        shooterIO_nl.stop();
        lastShooterTarget_RPM = Double.NaN;

        Logger.recordOutput("Launcher/Shooter/velTarget", Double.NaN, RPM);
        Logger.recordOutput("Launcher/Shooter/distTarget", Double.NaN, Meters);
    }

    public boolean isShooterAtTarget() {
        if (shooterIO_nl == null || !shooterInputs.connected) return false;
        if (Double.isNaN(lastShooterTarget_RPM)) return false;

        return MathUtil.isNear(
                lastShooterTarget_RPM, shooterInputs.vel_RPM, ShooterConstants.targetTolerance.asShooterRPM());
    }
}
