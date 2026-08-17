// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.brain.RobotBrain;
import frc.robot.brain.RobotState;
import frc.robot.config.Overrides;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.util.ControllerUtil;
import java.util.Optional;

public final class Robot extends TimedRobot {

    private static boolean _hasCreatedInstance = false;
    private static Robot _inst_nl = null;

    public static Robot instance() {
        if (!_hasCreatedInstance) {
            _hasCreatedInstance = true;
            _inst_nl = new Robot();
        } else if (_inst_nl == null) {
            throw new Error("Cannot access Robot instance within its own constructor!");
        }

        return _inst_nl;
    }

    public final RobotBrain brain;
    public final Field2d field;

    private double lastLoopTime = Double.NaN;
    private final DoubleWriter loopTimeWriter;

    private Robot() {
        Telemetry.init(this);
        Overrides.telemeterizeOverrides();
        RobotContainer.instance();

        brain = new RobotBrain();
        field = new Field2d();
        SmartDashboard.putData("field", field);

        loopTimeWriter = Telemetry.makeDoubleWriter("/", "loopTime", TelemetryUnits.seconds);
    }

    @Override
    public void robotPeriodic() {
        // track loop time
        double time = Timer.getFPGATimestamp();
        if (!Double.isNaN(lastLoopTime)) {
            loopTimeWriter.set(time - lastLoopTime);
        }
        lastLoopTime = time;

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
