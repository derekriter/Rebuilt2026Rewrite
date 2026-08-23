// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.HomeLauncher;
import frc.robot.commands.TeleopDrive;
import frc.robot.config.ControllerConfig;
import frc.robot.config.LEDsConfig;
import frc.robot.config.Overrides;
import frc.robot.pdh.PDH;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.shooter.IShooterIO;
import frc.robot.subsystems.launcher.shooter.ShooterIOReal;
import frc.robot.subsystems.launcher.shooter.ShooterIOSim;
import frc.robot.subsystems.launcher.turret.ITurretIO;
import frc.robot.subsystems.launcher.turret.TurretIOReal;
import frc.robot.subsystems.launcher.turret.TurretIOSim;
import frc.robot.subsystems.led.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class RobotContainer {

    private static boolean _hasCreatedInstance = false;
    private static RobotContainer _inst_nl = null;

    public static RobotContainer instance() {
        if (!_hasCreatedInstance) {
            _hasCreatedInstance = true;
            _inst_nl = new RobotContainer();
        } else if (_inst_nl == null) {
            throw new Error("Cannot access RobotContainer instance within its own constructor!");
        }

        return _inst_nl;
    }

    public final PDH pdh = new PDH();

    public final Swerve swerve = new Swerve();
    public final Launcher launcher;
    public final LEDs leds = new LEDs();

    public final CommandXboxController driver1Cmd = new CommandXboxController(ControllerConfig.driver1Port);
    public final XboxController driver1 = driver1Cmd.getHID();
    public final CommandXboxController driver2Cmd = new CommandXboxController(ControllerConfig.driver2Port);
    public final XboxController driver2 = driver2Cmd.getHID();

    private AutoFactory autoFactory;
    private LoggedDashboardChooser<Command> autoChooser;

    private RobotContainer() {
        switch (Mode.getMode()) {
            case REAL -> {
                launcher = new Launcher(
                        Overrides.disableTurret ? null : new TurretIOReal(),
                        Overrides.disableShooter ? null : new ShooterIOReal());
            }
            case SIM -> {
                launcher = new Launcher(
                        Overrides.disableTurret ? null : new TurretIOSim(),
                        Overrides.disableShooter ? null : new ShooterIOSim());
            }
            case REPLAY -> {
                launcher = new Launcher(
                        Overrides.disableTurret ? null : ITurretIO.blank,
                        Overrides.disableShooter ? null : IShooterIO.blank);
            }
                // shouldn't be possible to trigger, but the compiler required it anyways
            default -> throw new Error("Unhandled mode encountered");
        }

        setupControls();
        setupChoreo();
    }

    private void setupControls() {
        driver1Cmd.b().onTrue(seedSwerveOrientationCmd());

        driver2Cmd.x().onTrue(Commands.runOnce(() -> {
            Robot.instance().brain.state.isTurretHomed = false;
        }));
    }

    private void setupChoreo() {
        autoFactory = new AutoFactory(
                () -> swerve.getState().Pose,
                swerve::resetPose,
                swerve::followTrajectory,
                true,
                swerve,
                (Choreo.TrajectoryLogger<SwerveSample>) (Trajectory<SwerveSample> traj, Boolean isStart) -> {
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

    public Command autonCmd() {
        Command cmd = autoChooser.get();
        if (cmd == null) {
            return Commands.runOnce(() -> Console.println("No autonomous selected"))
                    .withName("Fallback auton");
        } else {
            return cmd;
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
                .withName("disconnLEDsCmd");
    }

    public Command idleLEDsCmd() {
        LEDPattern breathe = LEDPattern.solid(LEDsConfig.chargeGold).breathe(Seconds.of(2));
        return Commands.runEnd(() -> leds.applyPattern(breathe), leds::clear, leds)
                .ignoringDisable(true)
                .withName("idleLEDsCmd");
    }

    public Command autonLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(LEDsConfig.chargeGold)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("autonLEDsCmd");
    }

    public Command okLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(Color.kWhite)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("okLEDsCmd");
    }

    public Command errorLEDsCmd() {
        LEDPattern blink = LEDPattern.solid(Color.kRed).blink(Seconds.of(1 / 8.0));
        return Commands.runEnd(() -> leds.applyPattern(blink), leds::clear, leds)
                .ignoringDisable(true)
                .withName("errorLEDsCmd");
    }

    public Command endgameLEDsCmd() {
        LEDPattern rainbow =
                LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(4), LEDsConfig.ledSpacing);
        return Commands.runEnd(() -> leds.applyPattern(rainbow), leds::clear, leds)
                .ignoringDisable(true)
                .withName("endgameLEDsCmd");
    }

    public Command shiftWarningLEDsCmd() {
        LEDPattern blink = LEDPattern.solid(Color.kBlue).blink(Seconds.of(1 / 4.0));
        return Commands.runEnd(() -> leds.applyPattern(blink), leds::clear, leds)
                .ignoringDisable(true)
                .withName("shiftWarningLEDsCmd");
    }

    public Command hubActiveLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(LEDsConfig.chargeGreen)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("hubActiveLEDsCmd");
    }

    public Command hubInactiveLEDsCmd() {
        return Commands.startEnd(() -> leds.applyPattern(LEDPattern.solid(Color.kDimGray)), leds::clear, leds)
                .ignoringDisable(true)
                .withName("hubInactiveLEDsCmd");
    }

    public Command seedSwerveOrientationCmd() {
        return Commands.runOnce(swerve::seedFieldCentric).withName("seedSwerveOrientationCmd");
    }

    public Command stopSwerveCmd() {
        return Commands.startEnd(swerve::stop, () -> {}, swerve).withName("stopSwerveCmd");
    }

    public Command brakeSwerveCmd() {
        return Commands.startEnd(() -> swerve.setControl(new SwerveRequest.SwerveDriveBrake()), () -> {}, swerve)
                .withName("brakeSwerveCmd");
    }

    public Command deployIntakeCmd() {
        return Commands.runOnce(() -> Console.println("deploy intake") /*, intake*/)
                .withName("deployIntakeCmd");
    }

    public Command runIntakeCmd() {
        return Commands.startRun(() -> Console.println("run intake"), () -> {} /*, intake*/)
                .withName("runIntakeCmd");
    }

    public Command stopIntakeCmd() {
        return Commands.runOnce(() -> Console.println("stop intake") /*, intake*/)
                .withName("stopIntakeCmd");
    }

    public Command climbUpPosCmd() {
        return Commands.startRun(() -> Console.println("climb up"), () -> {} /*, climb*/)
                .withTimeout(1.6)
                .withName("climbUpPosCmd");
    }

    public Command climbHangingPosCmd() {
        return Commands.startRun(() -> Console.println("climb hanging"), () -> {} /*, climb*/)
                .withTimeout(1.4)
                .withName("climbHangingPosCmd");
    }
}
