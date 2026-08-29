// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoProgram;
import frc.robot.brain.RobotBrain;
import frc.robot.brain.RobotState;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.LauncherConstants.ShooterConstants;
import frc.robot.constants.LauncherConstants.TurretConstants;
import frc.robot.constants.Overrides;
import frc.robot.constants.PDHConstants;
import frc.robot.util.ControllerUtil;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Map;
import java.util.Optional;
import java.util.TimeZone;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public final class Robot extends LoggedRobot {

    private static Robot _inst_nl = null;

    public static Robot instance() {
        if (_inst_nl == null) {
            _inst_nl = new Robot();
        }

        return _inst_nl;
    }

    public final RobotBrain brain;
    public final Field2d field;
    public final CvSource setupImageFeed;
    public final Mat setupImage;

    private boolean showingAutoPath = false;
    private boolean showingAutoPathLast = false;
    private boolean autoPathNeedsUpdate = false;

    private Robot() {
        _inst_nl = this;

        initLogging();
        Overrides.telemeterizeOverrides();
        RobotContainer.instance();

        brain = new RobotBrain();
        field = new Field2d();
        SmartDashboard.putData("field", field);

        RobotContainer.instance().getAutoChooser().onChange(prog_nl -> {
            autoPathNeedsUpdate = true;
        });

        setupImageFeed = CameraServer.putVideo("setupImage", 320, 240);
        setupImage =
                Imgcodecs.imread(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "setupImages/test.jpg")
                        .toString());
    }

    private void initLogging() {
        Logger.recordMetadata("projectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("buildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("commitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("commitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("commitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "gitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "clean";
                    case 1 -> "dirty";
                    default -> "unknown";
                });
        Logger.recordMetadata("runtimeType", getRuntimeType().name());
        Logger.recordMetadata(
                "initDate",
                new SimpleDateFormat("dd MMM yyyy, hh:mm:ss a")
                        .format(Calendar.getInstance(TimeZone.getTimeZone("America/Detroit"))
                                .getTime()));

        switch (Mode.getMode()) {
            case REAL -> {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            }
            case REPLAY -> {
                setUseTiming(false);

                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            }
        }

        Logger.registerURCL(URCL.startExternal(Map.ofEntries(
                Map.entry(TurretConstants.canID, "turretMotor"),
                Map.entry(ShooterConstants.canID, "shooterMotor"),
                Map.entry(PDHConstants.canID, "PDH"))));
        StatusLogger.disableAutoLogging();

        DriverStation.silenceJoystickConnectionWarning(true);

        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        setupImageFeed.putFrame(setupImage);

        /*
         * Run command scheduler
         * First runs subsystem periodics, then scheduled commands
         *
         * Have to run before brain in order to make sure that inputs have been updated before subsystems are asked to report
         */
        RobotContainer.instance().pdh.periodic();
        CommandScheduler.getInstance().run();
        ControllerUtil.periodic(RobotContainer.instance().driver1, RobotContainer.instance().driver2);
        field.setRobotPose(RobotContainer.instance().swerve.getPose());

        brain.pollState();
        brain.determineModes();
        brain.log();
        brain.scheduleCommands();

        handleAutoPath();

        // update lastState in brain
        if (brain.lastState.isEmpty()) {
            brain.lastState = Optional.of(new RobotState());
        }
        brain.lastState.get().copyFrom(brain.state);
    }

    private void handleAutoPath() {
        if (showingAutoPath) {
            autoPathNeedsUpdate = autoPathNeedsUpdate
                    || !showingAutoPathLast
                    || brain.lastState.isPresent()
                            && (brain.state.isRed != brain.lastState.get().isRed
                                    || brain.state.isFMSAttached != brain.lastState.get().isFMSAttached);

            if (autoPathNeedsUpdate) {
                RobotContainer.instance().showAutonPath();
            }
        } else if (showingAutoPathLast) {
            AutoProgram.clearPathDisplay();
        }

        showingAutoPathLast = showingAutoPath;
        autoPathNeedsUpdate = false;
    }

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledInit() {
        showingAutoPath = true;
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        showingAutoPath = true;
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        showingAutoPath = false;
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        showingAutoPath = false;
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
