// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Mode;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.brain.OpMode;
import frc.robot.constants.Overrides;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.SwerveConstants.CANivoreConfig;
import frc.robot.constants.SwerveConstants.PigeonConfig;
import frc.robot.subsystems.swerve.IGyroIO.GyroIOInputs;
import frc.robot.util.AlertUtils;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

    // package private
    static final Lock odometryLock = new ReentrantLock();

    private final CANBus canBus;
    private final Alert canBusBreakerAlert = AlertUtils.makeBreakerTripAlert("canivore");
    private boolean canBusBreaker = false;
    private boolean canBusBreakerLast = false;

    private final IGyroIO gyroIO;
    private final GyroIOInputs gyroInputs = new GyroIOInputs();
    private final Alert gyroCANAlert = AlertUtils.makeCANFailureAlert("pigeon2");
    private final Alert gyroBreakerAlert = AlertUtils.makeBreakerTripAlert("pigeon2");
    private boolean gyroConnectedLast = false;
    private boolean gyroBreaker = false;
    private boolean gyroBreakerLast = false;

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

    private final PIDController trajXController = new PIDController(
            SwerveConstants.autonTranslationP, SwerveConstants.autonTranslationI, SwerveConstants.autonTranslationD);
    private final PIDController trajYController = new PIDController(
            SwerveConstants.autonTranslationP, SwerveConstants.autonTranslationI, SwerveConstants.autonTranslationD);
    private final PIDController trajThetaController =
            new PIDController(SwerveConstants.headingP, SwerveConstants.headingI, SwerveConstants.headingD);
    private boolean inTrajFollowingMode = false;
    private SwerveSample lastTrajSample_nl = null;

    public Swerve(
            CANBusDependentConstructor<IGyroIO> _gyroIO,
            CANBusDependentConstructor<IModuleIO> flModuleIO,
            CANBusDependentConstructor<IModuleIO> frModuleIO,
            CANBusDependentConstructor<IModuleIO> blModuleIO,
            CANBusDependentConstructor<IModuleIO> brModuleIO) {
        canBus = new CANBus(CANivoreConfig.busID);

        gyroIO = _gyroIO.construct(canBus);
        modules[0] = new Module(flModuleIO.construct(canBus), 0);
        modules[1] = new Module(frModuleIO.construct(canBus), 1);
        modules[2] = new Module(blModuleIO.construct(canBus), 2);
        modules[3] = new Module(brModuleIO.construct(canBus), 3);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.seedIsCANFD(canBus.isNetworkFD());
        PhoenixOdometryThread.instance().start();

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Swerve/sysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        trajThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("SwerveInputs/GyroInputs", gyroInputs);
        for (Module m : modules) {
            m.periodic();
        }
        odometryLock.unlock();

        if (Robot.instance().brain.state.opMode == OpMode.DISABLED) {
            // Stop moving when disabled
            inTrajFollowingMode = false;
            for (Module m : modules) {
                m.stop();
            }

            // Log empty setpoint states when disabled
            Logger.recordOutput("Swerve/targetStates", BlankValues.swerveModuleStateArray);
            Logger.recordOutput("Swerve/optimizedTargetStates", BlankValues.swerveModuleStateArray);
            Logger.recordOutput("Swerve/targetSpeeds", BlankValues.chassisSpeeds);
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps_s(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update canivore logging
        canBusBreaker = RobotContainer.instance().pdh.isBreakerTripped(CANivoreConfig.channelID);
        Logger.recordOutput("Swerve/CANivore/breakerTripped", canBusBreaker);
        canBusBreakerAlert.set(canBusBreaker);
        if (canBusBreaker != canBusBreakerLast) {
            if (canBusBreaker) {
                Console.reportBreakerTripNoCAN("canivore", CANivoreConfig.channelID);
            } else {
                Console.reportBreakerResetNoCAN("canivore", CANivoreConfig.channelID);
            }
        }

        // Update gyro logging
        gyroBreaker = RobotContainer.instance().pdh.isBreakerTripped(PigeonConfig.channelID);
        Logger.recordOutput("CAN/pigeon2_" + PigeonConfig.canID, gyroInputs.connected);
        gyroCANAlert.set(!gyroInputs.connected && !gyroBreaker && Mode.getMode() != Mode.SIM);
        if (gyroInputs.connected != gyroConnectedLast) {
            if (gyroInputs.connected) {
                Console.reportCANConnect("pigeon2", PigeonConfig.canID, PigeonConfig.channelID);
            } else {
                Console.reportCANDisconnect("pigeon2", PigeonConfig.canID, PigeonConfig.channelID);
            }
        }
        Logger.recordOutput("Swerve/Gyro/breakerTripped", gyroBreaker);
        gyroBreakerAlert.set(gyroBreaker);
        if (gyroBreaker != gyroBreakerLast) {
            if (gyroBreaker) {
                Console.reportBreakerTrip("pigeon2", PigeonConfig.canID, PigeonConfig.channelID);
            } else {
                Console.reportBreakerReset("pigeon2", PigeonConfig.canID, PigeonConfig.channelID);
            }
        }

        // Update general logging
        Logger.recordOutput("Swerve/measuredStates", getMeasuredModuleStates());
        Logger.recordOutput("Swerve/measuredSpeeds", getMeasuredRobotRelativeSpeeds());
        Logger.recordOutput("Swerve/robotPose", getPose());

        if (inTrajFollowingMode) {
            updateTrajFollowing();
        }
        Logger.recordOutput("Swerve/inTrajFollowingMode", inTrajFollowingMode);

        Command currentCommand = getCurrentCommand();
        Logger.recordOutput(
                "Swerve/currentCommand", currentCommand == null ? BlankValues.string : currentCommand.getName());

        Command defaultCommand = getDefaultCommand();
        Logger.recordOutput(
                "Swerve/defaultCommand", defaultCommand == null ? BlankValues.string : defaultCommand.getName());

        canBusBreakerLast = canBusBreaker;
        gyroConnectedLast = gyroInputs.connected;
        gyroBreakerLast = gyroBreaker;
    }

    private void updateTrajFollowing() {
        if (lastTrajSample_nl == null) {
            stop();
        } else {
            Pose2d currPose = getPose();

            ChassisSpeeds speeds = new ChassisSpeeds(
                    lastTrajSample_nl.vx + trajXController.calculate(currPose.getX(), lastTrajSample_nl.x),
                    lastTrajSample_nl.vy + trajYController.calculate(currPose.getY(), lastTrajSample_nl.y),
                    lastTrajSample_nl.omega
                            + trajThetaController.calculate(
                                    currPose.getRotation().getRadians(), lastTrajSample_nl.heading));

            runFieldRelativeVelocity(speeds);
            inTrajFollowingMode = true; // super janky way to prevent runVelocity from cancelling traj following mode
        }
    }

    public void report(SwerveReport report) {
        if (Overrides.disableSwerveSafety) {
            report.isOperational = true;
        } else {
            report.isOperational = modules[0].isOperational()
                    && modules[1].isOperational()
                    && modules[2].isOperational()
                    && modules[3].isOperational()
                    && ((gyroInputs.connected && !gyroBreaker) || Mode.getMode() == Mode.SIM);
        }
    }

    public void runRobotRelativeVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.speedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Swerve/targetStates", setpointStates);
        Logger.recordOutput("Swerve/targetSpeeds", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("Swerve/optimizedTargetStates", setpointStates);

        inTrajFollowingMode = false;
    }

    public void runFieldRelativeVelocity(ChassisSpeeds speeds) {
        runRobotRelativeVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation()));
    }

    public void runOperatorRelativeVelocity(ChassisSpeeds speeds) {
        runRobotRelativeVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, Robot.instance().brain.state.isRed ? getRotation().plus(Rotation2d.k180deg) : getRotation()));
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output_V) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output_V);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runRobotRelativeVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void followTrajectory(SwerveSample sample) {
        lastTrajSample_nl = sample;
        inTrajFollowingMode = true;
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    private SwerveModuleState[] getMeasuredModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getMeasuredModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    public ChassisSpeeds getMeasuredRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getMeasuredModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions_rad() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition_rad();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity_rps() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity_rps() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getMeasuredModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPose, double timestamp_s, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPose, timestamp_s, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeed_mps() {
        return SwerveConstants.speedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeed_radps() {
        return getMaxLinearSpeed_mps() / SwerveConstants.driveBaseRadius_m;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(
                    SwerveConstants.modules[0].constants.LocationX, SwerveConstants.modules[0].constants.LocationY),
            new Translation2d(
                    SwerveConstants.modules[1].constants.LocationX, SwerveConstants.modules[1].constants.LocationY),
            new Translation2d(
                    SwerveConstants.modules[2].constants.LocationX, SwerveConstants.modules[2].constants.LocationY),
            new Translation2d(
                    SwerveConstants.modules[3].constants.LocationX, SwerveConstants.modules[3].constants.LocationY)
        };
    }
}
