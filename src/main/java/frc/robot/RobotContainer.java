// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.HomeLauncher;
import frc.robot.commands.TeleopDrive;
import frc.robot.config.ControllerConfig;
import frc.robot.config.LEDsConfig;
import frc.robot.pdh.PDH;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.led.LEDs;
import frc.robot.subsystems.swerve.Swerve;

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
    public final Launcher launcher = new Launcher();
    public final LEDs leds = new LEDs();

    public final XboxController driver1 = new XboxController(ControllerConfig.driver1Port);
    public final XboxController driver2 = new XboxController(ControllerConfig.driver2Port);

    private RobotContainer() {}

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

    public Command teleopDriveCmd() {
        return new TeleopDrive(swerve);
    }
}
