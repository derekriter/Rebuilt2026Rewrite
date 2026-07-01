package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.config.LauncherConstants;
import frc.robot.config.LauncherConstants.ShooterConstants;
import frc.robot.config.LauncherConstants.TurretConstants;
import frc.robot.config.Overrides;
import frc.robot.subsystems.launcher.shooter.IShooterIO;
import frc.robot.subsystems.launcher.shooter.IShooterIO.ShooterIOInputs;
import frc.robot.subsystems.launcher.shooter.ShooterFlags;
import frc.robot.subsystems.launcher.shooter.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.launcher.shooter.ShooterTarget;
import frc.robot.subsystems.launcher.shooter.states.FixedTargetShooterState;
import frc.robot.subsystems.launcher.shooter.states.IShooterState;
import frc.robot.subsystems.launcher.shooter.states.InactiveShooterState;
import frc.robot.subsystems.launcher.shooter.states.TargetingPointShooterState;
import frc.robot.subsystems.launcher.turret.ITurretIO;
import frc.robot.subsystems.launcher.turret.ITurretIO.TurretIOInputs;
import frc.robot.subsystems.launcher.turret.TurretAngle;
import frc.robot.subsystems.launcher.turret.TurretFlags;
import frc.robot.subsystems.launcher.turret.TurretIOInputsAutoLogged;
import frc.robot.subsystems.launcher.turret.states.FixedAngleTurretState;
import frc.robot.subsystems.launcher.turret.states.ITurretState;
import frc.robot.subsystems.launcher.turret.states.InactiveTurretState;
import frc.robot.subsystems.launcher.turret.states.LockedTurretState;
import frc.robot.subsystems.launcher.turret.states.ManualControlTurretState;
import frc.robot.subsystems.launcher.turret.states.TargetPointTurretState;
import frc.robot.util.AlertUtils;
import java.util.Optional;
import java.util.function.Supplier;

public final class Launcher extends SubsystemBase {

    // ===Turret===
    private final ITurretIO turretIO;
    private TurretIOInputs turretInputs = new TurretIOInputsAutoLogged();

    private final Alert turretDisabled = AlertUtils.makeSystemDisabledAlert(TurretConstants.systemName),
            turretSafetyOff = AlertUtils.makeSafetyDisabledAlert(TurretConstants.systemName);
    private final Alert
            turretDisconnected = AlertUtils.makeDisconnectAlert(TurretConstants.motorName, TurretConstants.canID),
            turretOverheating = AlertUtils.makeDisconnectAlert(TurretConstants.motorName, TurretConstants.canID),
            turretCriticalOverheating =
                    AlertUtils.makeDisconnectAlert(TurretConstants.motorName, TurretConstants.canID),
            turretConfigFail = AlertUtils.makeConfigFailAlert(TurretConstants.motorName, TurretConstants.canID),
            turretHardwareFault = AlertUtils.makeHardwareFaultAlert(TurretConstants.motorName, TurretConstants.canID);

    private ITurretState reqTurretState = new InactiveTurretState();
    private ITurretState realTurretState = new InactiveTurretState();
    private Optional<ITurretState> lastRealTurretState = Optional.empty();

    private Optional<TurretFlags> turretFlags = Optional.empty();
    private boolean isTurretCalibrated = false;

    // ===Shooter===
    private final IShooterIO shooterIO;
    private ShooterIOInputs shooterInputs = new ShooterIOInputsAutoLogged();

    private final Alert shooterDisabled = AlertUtils.makeSystemDisabledAlert(ShooterConstants.systemName),
            shooterSafetyOff = AlertUtils.makeSafetyDisabledAlert(ShooterConstants.systemName);
    private final Alert
            shooterDisconnected = AlertUtils.makeDisconnectAlert(ShooterConstants.motorName, ShooterConstants.canID),
            shooterOverheating = AlertUtils.makeOverheatingAlert(ShooterConstants.motorName, ShooterConstants.canID),
            shooterCriticalOverheating =
                    AlertUtils.makeCriticalOverheatingAlert(ShooterConstants.motorName, ShooterConstants.canID),
            shooterConfigFail = AlertUtils.makeConfigFailAlert(ShooterConstants.motorName, ShooterConstants.canID),
            shooterHardwareFault =
                    AlertUtils.makeHardwareFaultAlert(ShooterConstants.motorName, ShooterConstants.canID);

    private IShooterState reqShooterState = new InactiveShooterState();
    private IShooterState realShooterState = new InactiveShooterState();
    private Optional<IShooterState> lastRealShooterState = Optional.empty();

    private Optional<ShooterFlags> shooterFlags = Optional.empty();

    public Launcher(ITurretIO _turretIO, IShooterIO _shooterIO) {
        // ===Turret===
        turretIO = _turretIO;

        turretDisabled.set(Overrides.disableTurret);
        turretSafetyOff.set(Overrides.disableTurretSafety);

        // ===Shooter===
        shooterIO = _shooterIO;

        shooterDisabled.set(Overrides.disableShooter);
        shooterSafetyOff.set(Overrides.disableShooterSafety);
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        updateTurretFlags();
        shooterIO.updateInputs(shooterInputs);
        updateShooterFlags();

        determineRealTurretState();
        controlTurret();

        determineRealShooterState();
        controlShooter();
    }

    private void updateTurretFlags() {
        boolean connected = turretInputs.connected;
        boolean overheating = turretInputs.temp.gte(Celsius.of(75));
        boolean critOverheating = turretInputs.temp.gte(Celsius.of(80));
        boolean configSuccess = turretInputs.motorConfigSuccessfull;
        boolean hardwareFaults = turretInputs.faultEscEEPROM
                || turretInputs.faultFirmware
                || turretInputs.faultGateDriver
                || turretInputs.faultSensor
                || turretInputs.faultMotorType;
        boolean overheatShutdown = (critOverheating
                        || turretFlags.map(flags -> flags.overheatShutdown()).orElse(false))
                && overheating;

        if (turretFlags.map(flags -> flags.motorConnected()).orElse(false) != connected) {
            if (connected) {
                System.out.printf("Connected to %s (CAN %d)\n", TurretConstants.motorName, TurretConstants.canID);
            } else {
                DriverStation.reportError(
                        String.format(
                                "Lost connection to %s (CAN %d)", TurretConstants.motorName, TurretConstants.canID),
                        false);
            }
        }
        if (turretFlags.isEmpty() && !configSuccess) {
            DriverStation.reportError(String.format("Failed to configure %s", TurretConstants.motorName), false);
            turretConfigFail.set(true);
        }

        turretFlags = Optional.of(new TurretFlags(
                connected, overheating, critOverheating, configSuccess, hardwareFaults, overheatShutdown));

        turretDisconnected.set(!connected);
        turretOverheating.set(overheating);
        turretCriticalOverheating.set(critOverheating);
        turretHardwareFault.set(hardwareFaults);
    }

    private void determineRealTurretState() {
        if (Overrides.disableTurret) {
            if (!(realTurretState instanceof LockedTurretState)) realTurretState = new LockedTurretState();
            return;
        } else if (Overrides.disableShooterSafety) {
            realShooterState = reqShooterState;
            return;
        }

        if (turretFlags.isEmpty()) {
            if (!(realTurretState instanceof InactiveTurretState)) realTurretState = new InactiveTurretState();
            return;
        }

        TurretFlags flags = turretFlags.get();
        if (!flags.motorConnected() || flags.overheatShutdown()) {
            if (!(realTurretState instanceof InactiveTurretState)) realTurretState = new InactiveTurretState();
        } else {
            realTurretState = reqTurretState;
        }
    }

    private void controlTurret() {
        boolean stateChanged =
                lastRealTurretState.isEmpty() || !lastRealTurretState.get().equals(realTurretState);

        if (stateChanged) {
            if (realTurretState instanceof LockedTurretState) {
                turretIO.setBrake(true);
            } else {
                turretIO.setBrake(!TurretConstants.coast);
            }
        }

        if (realTurretState instanceof LockedTurretState) {
            if (stateChanged) {
                turretIO.stop();
            }
        } else if (realTurretState instanceof FixedAngleTurretState) {
            if (stateChanged) {
                turretIO.runToPosition(((FixedAngleTurretState) realTurretState)
                        .angle()
                        .clampToLegalRange()
                        .asMotorAngle());
            }
        } else if (realTurretState instanceof ManualControlTurretState) {
            if (stateChanged) {
                turretIO.dutyCycle(((ManualControlTurretState) realTurretState).duty());
            }
        } else if (realTurretState instanceof TargetPointTurretState) {
            Pose2d robotPose = RobotContainer.instance().swerve.getPose();
            Translation2d launcherLoc = getLauncherLocOnField();
            Translation2d targetLoc =
                    ((TargetPointTurretState) realTurretState).point().get();
            Translation2d compTargetLoc = targetLoc.plus(calcPointCompensation());

            Translation2d vecToTarget = compTargetLoc.minus(launcherLoc);
            Angle fieldCentricAngle = Radians.of(Math.atan2(vecToTarget.getY(), vecToTarget.getX()));
            Angle robotCentricAngle =
                    fieldCentricAngle.minus(robotPose.getRotation().getMeasure());
            TurretAngle tAngle =
                    TurretAngle.fromMechanismAngle(robotCentricAngle).clampToLegalRange();

            turretIO.runToPosition(tAngle.asMotorAngle());
        } else {
            // turret is in inactive or invalid state

            if (stateChanged) {
                turretIO.stop();
            }
        }
    }

    private void updateShooterFlags() {
        boolean connected = shooterInputs.connected;
        boolean overheating = shooterInputs.temp.gte(Celsius.of(75));
        boolean critOverheating = shooterInputs.temp.gte(Celsius.of(80));
        boolean configSuccess = shooterInputs.motorConfigSuccessfull;
        boolean hardwareFaults = shooterInputs.faultEscEEPROM
                || shooterInputs.faultFirmware
                || shooterInputs.faultGateDriver
                || shooterInputs.faultSensor
                || shooterInputs.faultMotorType;
        boolean overheatShutdown = (critOverheating
                        || shooterFlags.map(flags -> flags.overheatShutdown()).orElse(false))
                && overheating;

        if (shooterFlags.map(flags -> flags.motorConnected()).orElse(false) != connected) {
            if (connected) {
                System.out.printf("Connected to %s (CAN %d)\n", ShooterConstants.motorName, ShooterConstants.canID);
            } else {
                DriverStation.reportError(
                        String.format(
                                "Lost connection to %s (CAN %d)", ShooterConstants.motorName, ShooterConstants.canID),
                        false);
            }
        }
        if (shooterFlags.isEmpty() && !configSuccess) {
            DriverStation.reportError(String.format("Failed to configure %s", ShooterConstants.motorName), false);
            shooterConfigFail.set(true);
        }

        shooterFlags = Optional.of(new ShooterFlags(
                connected, overheating, critOverheating, configSuccess, hardwareFaults, overheatShutdown));

        shooterDisconnected.set(!connected);
        shooterOverheating.set(overheating);
        shooterCriticalOverheating.set(critOverheating);
        shooterHardwareFault.set(hardwareFaults);
    }

    private void determineRealShooterState() {
        if (Overrides.disableShooter) {
            if (!(realShooterState instanceof InactiveShooterState)) realShooterState = new InactiveShooterState();
            return;
        } else if (Overrides.disableShooterSafety) {
            realShooterState = reqShooterState;
            return;
        }

        if (shooterFlags.isEmpty()) {
            if (!(realShooterState instanceof InactiveShooterState)) realShooterState = new InactiveShooterState();
            return;
        }

        ShooterFlags flags = shooterFlags.get();
        if (!flags.motorConnected() || flags.overheatShutdown()) {
            if (!(realShooterState instanceof InactiveShooterState)) realShooterState = new InactiveShooterState();
        } else {
            realShooterState = reqShooterState;
        }
    }

    private void controlShooter() {
        boolean stateChanged =
                lastRealShooterState.isEmpty() || !lastRealShooterState.get().equals(realShooterState);

        if (realShooterState instanceof FixedTargetShooterState) {
            if (stateChanged) {
                shooterIO.runToVelocity(((FixedTargetShooterState) realShooterState)
                        .target()
                        .clampToLegalRange()
                        .asShooterVelocity());
            }
        } else if (realShooterState instanceof TargetingPointShooterState) {
            Translation2d launcherLoc = getLauncherLocOnField();
            Translation2d targetLoc =
                    ((TargetingPointShooterState) realShooterState).point().get();
            Translation2d compTargetLoc = targetLoc.plus(calcPointCompensation());

            Distance dist = Meters.of(launcherLoc.getDistance(compTargetLoc));
            ShooterTarget sTarget = ShooterTarget.fromDistanceToTarget(dist).clampToLegalRange();

            shooterIO.runToVelocity(sTarget.asShooterVelocity());
        } else {
            // shooter is in inactive or invalid state

            if (stateChanged) {
                shooterIO.stop();
            }
        }
    }

    private Translation2d getLauncherLocOnField() {
        Pose2d robot = RobotContainer.instance().swerve.getPose();
        return robot.getTranslation().plus(LauncherConstants.launcherOffset.rotateBy(robot.getRotation()));
    }

    private Translation2d calcPointCompensation() {
        Pose2d robotPose = RobotContainer.instance().swerve.getPose();
        Pair<LinearVelocity, LinearVelocity> robotVel =
                RobotContainer.instance().swerve.getRealFieldRelativeVelocity();
        Translation2d velCompensation = new Translation2d(
                        robotVel.getFirst().times(LauncherConstants.ballAirTime),
                        robotVel.getSecond().times(LauncherConstants.ballAirTime))
                .rotateBy(robotPose.getRotation())
                .unaryMinus();

        return velCompensation;
    }

    public void targetPoint(Supplier<Translation2d> point) {
        reqShooterState = new TargetingPointShooterState(point);
        reqTurretState = new TargetPointTurretState(point);
    }

    public void fixedTargetShooter(ShooterTarget target) {
        if (!target.isLegal()) {
            DriverStation.reportWarning("Illegal shooter target will be clamped", true);
        }

        reqShooterState = new FixedTargetShooterState(target.clampToLegalRange());
    }

    public void fixedAngleTurret(TurretAngle angle) {
        if (!angle.isLegal()) {
            DriverStation.reportWarning("Illegal turret target will be clamped", true);
        }

        reqTurretState = new FixedAngleTurretState(angle.clampToLegalRange());
    }

    public void stop() {
        reqShooterState = new InactiveShooterState();
        reqTurretState = new InactiveTurretState();
    }

    public void stopTurret() {
        reqTurretState = new InactiveTurretState();
    }

    public void stopShooter() {
        reqShooterState = new InactiveShooterState();
    }

    public void manualTurret(double duty) {
        reqTurretState = new ManualControlTurretState(duty);
    }

    public boolean getIsTurretCalibrated() {
        return isTurretCalibrated;
    }
}
