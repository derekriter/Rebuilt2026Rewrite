// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.brain.RobotBrain;
import frc.robot.brain.RobotState;
import java.util.Optional;

public final class Robot extends TimedRobot {

    public final RobotContainer container;
    public RobotState state;
    public Optional<RobotState> lastState;
    public final RobotBrain brain;

    public Robot() {
        initLogging();

        container = new RobotContainer();

        lastState = Optional.empty();
        state = new RobotState();
        state.isReal = isReal();

        brain = new RobotBrain(this);
    }

    private void initLogging() {}

    @Override
    public void robotPeriodic() {
        brain.pollState();
        brain.determineModes();
        brain.runCommands();

        /*
         * Run command scheduler
         * First runs subsystem periodics, then scheduled commands
         */
        CommandScheduler.getInstance().run();

        // update lastState
        if (lastState.isEmpty()) {
            lastState = Optional.of(new RobotState());
        }
        lastState.get().copyFrom(state);
    }

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
