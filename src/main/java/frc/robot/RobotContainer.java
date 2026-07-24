// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public final Swerve swerve;

    private RobotContainer() {
        swerve = new Swerve();
    }
}
