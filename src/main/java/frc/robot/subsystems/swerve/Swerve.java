// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE_AdvantageKit file
// at the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Mode;
import frc.robot.Robot;
import frc.robot.config.Overrides;
import frc.robot.config.SwerveConfig;
import frc.robot.util.AlertUtils;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

    static final Lock odometryLock = new ReentrantLock();
    private final IGyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    // private final Alert gyroDisconnectedAlert =
    //         new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    private final Alert gyroDisconnectedAlert = AlertUtils.makeCANFailureAlert("pigeon2");

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
            SwerveConfig.autonTranslationP, SwerveConfig.autonTranslationI, SwerveConfig.autonTranslationD);
    private final PIDController trajYController = new PIDController(
            SwerveConfig.autonTranslationP, SwerveConfig.autonTranslationI, SwerveConfig.autonTranslationD);
    private final PIDController trajThetaController =
            new PIDController(SwerveConfig.headingP, SwerveConfig.headingI, SwerveConfig.headingD);
    private boolean inTrajFollowingMode = false;
    private SwerveSample lastTrajSample_nl = null;

    public Swerve(
            IGyroIO gyroIO, IModuleIO flModuleIO, IModuleIO frModuleIO, IModuleIO blModuleIO, IModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Swerve/sysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("SwerveInputs/GyroInputs", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            inTrajFollowingMode = false;

            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Swerve/targetStates", new SwerveModuleState[] {});
            Logger.recordOutput("Swerve/optimizedTargetStates", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
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

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Mode.getMode() != Mode.SIM);

        Logger.recordOutput("Swerve/measuredStates", getMeasuredModuleStates());
        Logger.recordOutput("Swerve/measuredSpeeds", getMeasuredRobotRelativeSpeeds());
        Logger.recordOutput("Swerve/robotPose", getPose());

        if (inTrajFollowingMode) {
            updateTrajFollowing();
        }
        Logger.recordOutput("Swerve/inTrajFollowingMode", inTrajFollowingMode);

        Command currentCommand = getCurrentCommand();
        Logger.recordOutput("Swerve/currentCommand", currentCommand == null ? null : currentCommand.getName());

        Command defaultCommand = getDefaultCommand();
        Logger.recordOutput("Swerve/defaultCommand", defaultCommand == null ? null : defaultCommand.getName());
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

            runRobotRelativeVelocity(speeds);
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
                    && (gyroInputs.connected || Mode.getMode() == Mode.SIM);
        }
    }

    public void runRobotRelativeVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConfig.speedAt12Volts);

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
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
    }

    public void runOperatorRelativeVelocity(ChassisSpeeds speeds) {
        runRobotRelativeVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, Robot.instance().brain.state.isRed ? getRotation().plus(Rotation2d.k180deg) : getRotation()));
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
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
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
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
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return SwerveConfig.speedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / SwerveConfig.driveBaseRadius_m;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(SwerveConfig.modules[0].constants.LocationX, SwerveConfig.modules[0].constants.LocationY),
            new Translation2d(SwerveConfig.modules[1].constants.LocationX, SwerveConfig.modules[1].constants.LocationY),
            new Translation2d(SwerveConfig.modules[2].constants.LocationX, SwerveConfig.modules[2].constants.LocationY),
            new Translation2d(SwerveConfig.modules[3].constants.LocationX, SwerveConfig.modules[3].constants.LocationY)
        };
    }
}
