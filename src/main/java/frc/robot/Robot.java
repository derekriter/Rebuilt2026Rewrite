// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.brain.RobotBrain;
import frc.robot.brain.RobotState;
import frc.robot.config.Overrides;
import frc.robot.telemetry.Telemetry;
import frc.robot.util.ControllerUtil;
import java.util.Optional;

public final class Robot extends TimedRobot {

    private static boolean _hasCreatedInstance = false;
    private static Robot _inst = null;

    public static Robot instance() {
        if (!_hasCreatedInstance) {
            _hasCreatedInstance = true;
            _inst = new Robot();
        } else if (_inst == null) {
            throw new Error("Cannot access Robot instance within its own constructor!");
        }

        return _inst;
    }

    public final RobotBrain brain;

    private Robot() {
        Telemetry.init(this);
        Overrides.telemeterizeOverrides();
        RobotContainer.instance();

        brain = new RobotBrain();
    }

    @Override
    public void robotPeriodic() {
        brain.pollState();
        brain.determineModes();
        RobotBrain.robotStateWriter.set(brain.state);
        brain.runCommands();

        /*
         * Run command scheduler
         * First runs subsystem periodics, then scheduled commands
         */
        CommandScheduler.getInstance().run();
        ControllerUtil.periodic(RobotContainer.instance().driver1, RobotContainer.instance().driver2);

        // update lastState in brain
        if (brain.lastState.isEmpty()) {
            brain.lastState = Optional.of(new RobotState());
        }
        brain.lastState.get().copyFrom(brain.state);
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
