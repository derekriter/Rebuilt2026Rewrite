// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoProgram;
import frc.robot.auto.Autos;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.HomeLauncher;
import frc.robot.commands.TeleopDrive;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.IntakeConstants.RollerConstants;
import frc.robot.constants.LEDsConstants;
import frc.robot.constants.Overrides;
import frc.robot.pdh.PDH;
import frc.robot.subsystems.intake.IIntakeIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.shooter.IShooterIO;
import frc.robot.subsystems.launcher.shooter.ShooterIOReal;
import frc.robot.subsystems.launcher.shooter.ShooterIOSim;
import frc.robot.subsystems.launcher.turret.ITurretIO;
import frc.robot.subsystems.launcher.turret.TurretIOReal;
import frc.robot.subsystems.launcher.turret.TurretIOSim;
import frc.robot.subsystems.led.LEDs;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.IGyroIO;
import frc.robot.subsystems.swerve.IModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class RobotContainer {

    private static RobotContainer _inst_nl = null;

    public static RobotContainer instance() {
        if (_inst_nl == null) {
            _inst_nl = new RobotContainer();
        }

        return _inst_nl;
    }

    public final PDH pdh = new PDH();

    public final Swerve swerve;
    public final Launcher launcher;
    public final LEDs leds = new LEDs();
    public final Intake intake;

    public final CommandXboxController driver1Cmd = new CommandXboxController(ControllerConstants.driver1Port);
    public final XboxController driver1 = driver1Cmd.getHID();
    public final CommandXboxController driver2Cmd = new CommandXboxController(ControllerConstants.driver2Port);
    public final XboxController driver2 = driver2Cmd.getHID();

    private AutoFactory autoFactory;
    private LoggedDashboardChooser<AutoProgram> autoChooser;
    private Alert debugAutoOnFieldAlert =
            new Alert("A debug auto has been selected while the FMS is collected", AlertType.kWarning);

    private RobotContainer() {
        _inst_nl = this;

        switch (Mode.getMode()) {
            case REAL -> {
                swerve = new Swerve(
                        bus -> new GyroIOPigeon2(bus),
                        bus -> new ModuleIOTalonFX(0, bus),
                        bus -> new ModuleIOTalonFX(1, bus),
                        bus -> new ModuleIOTalonFX(2, bus),
                        bus -> new ModuleIOTalonFX(3, bus));
                launcher = new Launcher(
                        Overrides.disableTurret ? null : new TurretIOReal(),
                        Overrides.disableShooter ? null : new ShooterIOReal());
                intake = new Intake(Overrides.disableIntake ? null : new IntakeIOReal());
            }
            case SIM -> {
                swerve = new Swerve(
                        bus -> IGyroIO.blank,
                        bus -> new ModuleIOSim(0),
                        bus -> new ModuleIOSim(1),
                        bus -> new ModuleIOSim(2),
                        bus -> new ModuleIOSim(3));
                launcher = new Launcher(
                        Overrides.disableTurret ? null : new TurretIOSim(),
                        Overrides.disableShooter ? null : new ShooterIOSim());
                intake = new Intake(Overrides.disableIntake ? null : new IntakeIOSim());
            }
            case REPLAY -> {
                swerve = new Swerve(
                        bus -> IGyroIO.blank,
                        bus -> IModuleIO.blank,
                        bus -> IModuleIO.blank,
                        bus -> IModuleIO.blank,
                        bus -> IModuleIO.blank);
                launcher = new Launcher(
                        Overrides.disableTurret ? null : ITurretIO.blank,
                        Overrides.disableShooter ? null : IShooterIO.blank);
                intake = new Intake(Overrides.disableIntake ? null : IIntakeIO.blank);
            }
                // shouldn't be possible to trigger, but the compiler required it anyway
            default -> throw new Error("Unhandled mode encountered");
        }

        setupControls();
        setupChoreo();
    }

    private void setupControls() {
        driver1Cmd.b().onTrue(seedSwerveOrientationCmd());

        driver2Cmd.x().onTrue(homeLauncherCmd());
        driver2Cmd.leftTrigger().and(driver2Cmd.rightTrigger().negate()).whileTrue(intakeCmd());
        driver2Cmd
                .leftBumper()
                .and(driver2Cmd.rightTrigger().negate())
                .and(driver2Cmd.leftTrigger().negate())
                .whileTrue(reverseIntakeCmd());
    }

    private void setupChoreo() {
        Logger.recordOutput("Choreo/trajectory", BlankValues.pose2dArray);
        Logger.recordOutput("Choreo/trajectoryName", BlankValues.string);
        Logger.recordOutput("Choreo/trajectoryTime", Double.NaN, Seconds.name());

        autoFactory = new AutoFactory(
                swerve::getPose, swerve::setPose, swerve::followTrajectory, true, swerve, (Choreo.TrajectoryLogger<
                                SwerveSample>)
                        (Trajectory<SwerveSample> traj, Boolean isStart) -> {
                            if (Robot.instance().brain.state.isRed) {
                                traj = traj.flipped();
                            }

                            if (isStart) {
                                Logger.recordOutput("Choreo/trajectory", traj.getPoses());
                                Logger.recordOutput("Choreo/trajectoryName", traj.name());
                                Logger.recordOutput("Choreo/trajectoryTime", traj.getTotalTime(), Seconds.name());
                            } else {
                                Logger.recordOutput("Choreo/trajectory", BlankValues.pose2dArray);
                                Logger.recordOutput("Choreo/trajectoryName", BlankValues.string);
                                Logger.recordOutput("Choreo/trajectoryTime", Double.NaN, Seconds.name());
                            }
                        });

        autoChooser = Autos.createAutos(autoFactory);
    }

    public LoggedDashboardChooser<AutoProgram> getAutoChooser() {
        return autoChooser;
    }

    public void showAutonPath() {
        AutoProgram prog_nl = autoChooser.get();

        if (prog_nl == null) {
            AutoProgram.clearPathDisplay();
            debugAutoOnFieldAlert.set(false);
        } else {
            prog_nl.displayPath();
            debugAutoOnFieldAlert.set(Robot.instance().brain.state.isFMSAttached && prog_nl.getIsDebug());
        }
    }

    public void pushAutonRefFrame() {
        AutoProgram prog_nl = autoChooser.get();

        if (prog_nl == null) {
            AutoProgram.clearSetupReference();
        } else {
            prog_nl.displaySetupReference();
        }
    }

    public Command autonCmd() {
        AutoProgram prog = autoChooser.get();
        if (prog == null) {
            return Commands.runOnce(() -> Console.println("No autonomous selected"))
                    .withName("Fallback auton");
        } else {
            return prog.getCommand();
        }
    }

    public Command teleopDriveCmd() {
        return new TeleopDrive(swerve);
    }

    public HomeLauncher homeLauncherCmd() {
        return new HomeLauncher(launcher);
    }

    public AimAtTarget aimAtHubCmd() {
        return AimAtTarget.atHub(launcher, swerve, Robot.instance().brain.state.isRed, () -> Robot.instance()
                .brain
                .state
                .overrideTurret);
    }

    public AimAtTarget aimAtFZoneCmd() {
        return AimAtTarget.atFZone(launcher, swerve, Robot.instance().brain.state.isRed, () -> Robot.instance()
                .brain
                .state
                .overrideTurret);
    }

    public Command disconnLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(Color.kRed)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("disconnLEDs");
    }

    public Command idleLEDsCmd() {
        LEDPattern breathe = LEDPattern.solid(LEDsConstants.chargeGold).breathe(Seconds.of(2));
        return Commands.runEnd(() -> leds.applyPattern(breathe), leds::clear, leds)
                .ignoringDisable(true)
                .withName("idleLEDs");
    }

    public Command autonLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(LEDsConstants.chargeGold)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("autonLEDs");
    }

    public Command okLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(Color.kWhite)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("okLEDs");
    }

    public Command errorLEDsCmd() {
        LEDPattern blink = LEDPattern.solid(Color.kRed).blink(Seconds.of(1 / 8.0));
        return Commands.runEnd(() -> leds.applyPattern(blink), leds::clear, leds)
                .ignoringDisable(true)
                .withName("errorLEDs");
    }

    public Command endgameLEDsCmd() {
        LEDPattern rainbow =
                LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(4), LEDsConstants.ledSpacing);
        return Commands.runEnd(() -> leds.applyPattern(rainbow), leds::clear, leds)
                .ignoringDisable(true)
                .withName("endgameLEDs");
    }

    public Command shiftWarningLEDsCmd() {
        LEDPattern blink = LEDPattern.solid(Color.kBlue).blink(Seconds.of(1 / 4.0));
        return Commands.runEnd(() -> leds.applyPattern(blink), leds::clear, leds)
                .ignoringDisable(true)
                .withName("shiftWarningLEDs");
    }

    public Command hubActiveLEDsCmd() {
        return Commands.startEnd(
                        () -> leds.applyPattern(LEDPattern.solid(LEDsConstants.chargeGreen)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("hubActiveLEDs");
    }

    public Command hubInactiveLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(Color.kDimGray)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("hubInactiveLEDs");
    }

    public Command seedSwerveOrientationCmd() {
        return Commands.runOnce(
                        () -> swerve.setPose(new Pose2d(
                                swerve.getPose().getTranslation(),
                                Robot.instance().brain.state.isRed ? Rotation2d.k180deg : Rotation2d.kZero)),
                        swerve)
                .ignoringDisable(true)
                .withName("seedSwerveOrientation");
    }

    public Command stopSwerveCmd() {
        return Commands.startEnd(swerve::stop, () -> {}, swerve)
                .ignoringDisable(true)
                .withName("stopSwerve");
    }

    public Command brakeSwerveCmd() {
        return Commands.startEnd(swerve::stopWithX, () -> {}, swerve).withName("brakeSwerve");
    }

    public Command deployIntakeCmd() {
        return Commands.runOnce(intake::deploy, intake).withName("deployIntake");
    }

    public Command intakeCmd() {
        return Commands.startEnd(
                        () -> intake.setRollerVoltage(RollerConstants.intakeVoltage.in(Volts)),
                        intake::stopRoller,
                        intake)
                .withName("intake");
    }

    public Command reverseIntakeCmd() {
        return Commands.startEnd(
                        () -> intake.setRollerVoltage(RollerConstants.reverseVoltage.in(Volts)),
                        intake::stopRoller,
                        intake)
                .withName("reverseIntake");
    }

    public Command climbUpPosCmd() {
        return Commands.startRun(() -> Console.println("climb up"), () -> {} /*, climb*/)
                .withTimeout(1.6)
                .withName("climbUpPos");
    }

    public Command climbHangingPosCmd() {
        return Commands.startRun(() -> Console.println("climb hanging"), () -> {} /*, climb*/)
                .withTimeout(1.4)
                .withName("climbHangingPos");
    }
}
