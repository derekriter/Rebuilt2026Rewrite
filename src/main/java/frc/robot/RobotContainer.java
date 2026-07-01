// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.config.RobotMode;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.shooter.IShooterIO;
import frc.robot.subsystems.launcher.shooter.SimShooterIO;
import frc.robot.subsystems.launcher.shooter.SparkFlexShooterIO;
import frc.robot.subsystems.launcher.turret.ITurretIO;
import frc.robot.subsystems.launcher.turret.SimTurretIO;
import frc.robot.subsystems.launcher.turret.SparkMAXTurretIO;
import frc.robot.subsystems.swerve.Swerve;

public final class RobotContainer {

    private static RobotContainer inst = null;

    public static RobotContainer instance() {
        if (inst == null) {
            inst = new RobotContainer();
        }
        return inst;
    }

    public final Launcher launcher;
    public final Swerve swerve;

    private RobotContainer() {
        switch (RobotMode.currentMode) {
            case REAL:
            default:
                launcher = new Launcher(new SparkMAXTurretIO(), new SparkFlexShooterIO());
                swerve = new Swerve();
                break;
            case SIM:
                launcher = new Launcher(new SimTurretIO(), new SimShooterIO());
                swerve = new Swerve();
                break;
            case REPLAY:
                launcher = new Launcher(ITurretIO.blank, IShooterIO.blank);
                swerve = new Swerve();
                break;
        }
    }
}
