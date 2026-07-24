// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.state.RobotState;
import java.util.Optional;

public final class Robot extends TimedRobot {

    private RobotState robotState;
    private Optional<RobotState> lastRobotState;
    private RobotLogic robotLogic;

    public Robot() {
        initLogging();
        RobotContainer.instance();

        lastRobotState = Optional.empty();
        robotState = new RobotState();
        robotState.isReal = isReal();

        robotLogic = new RobotLogic(robotState, lastRobotState);
    }

    private void initLogging() {}

    @Override
    public void robotPeriodic() {
        // update state
        robotLogic.updateState();

        // run respective exit function(s) if necessary
        if (lastRobotState.isPresent() && robotState.robotMode != lastRobotState.get().robotMode) {
            switch (lastRobotState.get().robotMode) {
                case DISABLED -> robotLogic.disabledExit();
                case AUTON -> robotLogic.autonomousExit();
                case TELEOP -> robotLogic.teleopExit();
                case TEST -> robotLogic.testExit();
            }
            if (!robotState.robotMode.enabled) robotLogic.enabledExit();
        }
        // run respective init function(s) if necessary
        if (lastRobotState.isEmpty() || robotState.robotMode != lastRobotState.get().robotMode) {
            if (robotState.robotMode.enabled) robotLogic.enabledInit();
            switch (robotState.robotMode) {
                case DISABLED -> robotLogic.disabledInit();
                case AUTON -> robotLogic.autonomousInit();
                case TELEOP -> robotLogic.teleopInit();
                case TEST -> robotLogic.testInit();
            }
        }

        // run first periodic
        robotLogic.firstPeriodic();

        // run appropriate periodic(s) for robot mode
        if (robotState.robotMode.enabled) robotLogic.enabledPeriodic();
        switch (robotState.robotMode) {
            case DISABLED -> robotLogic.disabledPeriodic();
            case AUTON -> robotLogic.autonomousPeriodic();
            case TELEOP -> robotLogic.teleopPeriodic();
            case TEST -> robotLogic.testPeriodic();
        }

        // run final periodic
        robotLogic.finalPeriodic();

        /*
         * Run command scheduler
         * First runs subsystem periodics, then scheduled commands
         */
        CommandScheduler.getInstance().run();

        // update lastRobotState
        if (lastRobotState.isEmpty()) {
            lastRobotState = Optional.of(new RobotState());
        }
        lastRobotState.get().copyFrom(robotState);
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
