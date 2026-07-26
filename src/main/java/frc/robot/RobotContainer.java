// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.HomeLauncher;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.swerve.Swerve;

public final class RobotContainer {

    public final Swerve swerve = new Swerve();
    public final Launcher launcher = new Launcher();

    public final HomeLauncher homeLauncherCmd = new HomeLauncher(launcher);

    public RobotContainer() {}
}
