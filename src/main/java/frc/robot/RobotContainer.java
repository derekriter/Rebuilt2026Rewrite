// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.HomeLauncher;
import frc.robot.config.ControllerConfig;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.Swerve;

public final class RobotContainer {

    private static boolean _hasCreatedInstance = false;
    private static RobotContainer _inst = null;

    public static RobotContainer instance() {
        if (!_hasCreatedInstance) {
            _hasCreatedInstance = true;
            _inst = new RobotContainer();
        } else if (_inst == null) {
            throw new Error("Cannot access RobotContainer instance within its own constructor!");
        }

        return _inst;
    }

    public final Swerve swerve = new Swerve();
    public final Launcher launcher = new Launcher();

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
}
