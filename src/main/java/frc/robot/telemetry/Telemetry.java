package frc.robot.telemetry;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.brain.RobotState;
import frc.robot.config.TelemetryConfig;
import frc.robot.constants.BuildConstants;
import frc.robot.subsystems.launcher.LauncherReport;
import frc.robot.subsystems.launcher.ShooterTarget;
import frc.robot.subsystems.launcher.TurretAngle;
import frc.robot.telemetry.writer.BoolArrayWriter;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleArrayWriter;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.telemetry.writer.LongArrayWriter;
import frc.robot.telemetry.writer.LongWriter;
import frc.robot.telemetry.writer.RawWriter;
import frc.robot.telemetry.writer.StringArrayWriter;
import frc.robot.telemetry.writer.StringWriter;
import frc.robot.telemetry.writer.StructArrayWriter;
import frc.robot.telemetry.writer.StructWriter;
import frc.robot.telemetry.writer.compound.LauncherReportWriter;
import frc.robot.telemetry.writer.compound.RobotStateWriter;
import frc.robot.telemetry.writer.compound.ShooterTargetWriter;
import frc.robot.telemetry.writer.compound.SubsystemWriter;
import frc.robot.telemetry.writer.compound.TurretAngleWriter;
import frc.robot.util.CloneOperation;
import frc.robot.util.EqualityTest;
import java.lang.reflect.Field;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Calendar;
import java.util.TimeZone;

public class Telemetry {

    private static final Alert flashdriveNotRecognizedAlert =
            new Alert("Logging flashdrive not recognized", AlertType.kError);

    private static boolean hasInited = false;

    private static final Pose2d[] pose2dArrayNullFallback = new Pose2d[0];
    private static final Pose3d[] pose3dArrayNullFallback = new Pose3d[0];
    private static final ChassisSpeeds chassisSpeedsNullFallback =
            new ChassisSpeeds(Double.NaN, Double.NaN, Double.NaN);
    private static final SwerveModuleState swerveModuleStateNullFallback =
            new SwerveModuleState(Double.NaN, new Rotation2d(Double.NaN));
    private static final SwerveModuleState[] swerveModuleStateArrayNullFallback = new SwerveModuleState[0];
    private static final SwerveModulePosition swerveModulePositionNullFallback =
            new SwerveModulePosition(Double.NaN, new Rotation2d(Double.NaN));
    private static final SwerveModulePosition[] swerveModulePositionArrayNullFallback = new SwerveModulePosition[0];
    private static final RobotState robotStateNullFallback = new RobotState();
    private static final LauncherReport launcherReportNullFallback = new LauncherReport();
    private static final TurretAngle turretAngleNullFallback = TurretAngle.fromMechanismDeg(Double.NaN);
    private static final ShooterTarget shooterTargetAngleNullFallback = ShooterTarget.fromShooterRPM(Double.NaN);

    public static void init(Robot robot) {
        if (hasInited) return;

        // Adjust loop overrun warning timeout
        if (TelemetryConfig.loopOverrunPeriod.isPresent()) {
            try {
                Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
                watchdogField.setAccessible(true);
                Watchdog watchdog = (Watchdog) watchdogField.get(robot);
                watchdog.setTimeout(TelemetryConfig.loopOverrunPeriod.get().in(Seconds));

                CommandScheduler.getInstance()
                        .setPeriod(TelemetryConfig.loopOverrunPeriod.get().in(Seconds));
            } catch (Exception e) {
                reportWarning(TelemetryConfig.PREFIX + "Failed to adjust loop overrun warnings", false);
            }
        }
        DriverStation.silenceJoystickConnectionWarning(!TelemetryConfig.showJoystickDisconnectWarnings);
        SignalLogger.enableAutoLogging(TelemetryConfig.ctreLoggingEnabled);
        if (!TelemetryConfig.revLoggingEnabled) {
            StatusLogger.disableAutoLogging();
        }

        if (TelemetryConfig.telemetryLevel.logToFile) {
            DataLogManager.start();
            DataLogManager.logConsoleOutput(true);
            DataLogManager.logNetworkTables(true);
            DriverStation.startDataLog(DataLogManager.getLog(), true);

            // diagnostics
            try (StringWriter logDirectoryWriter = makeStringWriterEx("Telem", "logDirectory", null, false, true);
                    BoolWriter recognizedWriter =
                            makeBoolWriterEx("Telem", "flashdriveRecognized", null, false, true)) {
                String logDir = DataLogManager.getLogDir();

                logDirectoryWriter.set(logDir);

                boolean loggingToFlash =
                        logDir == null ? false : logDir.toLowerCase().startsWith("/u");
                recognizedWriter.set(loggingToFlash);
                flashdriveNotRecognizedAlert.set(!loggingToFlash);
            }
        }

        // metadata
        makeStringWriterInitialEx(
                        "Metadata",
                        "projectName",
                        null,
                        BuildConstants.MAVEN_NAME == null ? "null" : BuildConstants.MAVEN_NAME,
                        false,
                        true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata",
                        "buildDate",
                        null,
                        BuildConstants.BUILD_DATE == null ? "null" : BuildConstants.BUILD_DATE,
                        false,
                        true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata",
                        "commitSHA",
                        null,
                        BuildConstants.GIT_SHA == null ? "null" : BuildConstants.GIT_SHA,
                        false,
                        true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata",
                        "commitDate",
                        null,
                        BuildConstants.GIT_DATE == null ? "null" : BuildConstants.GIT_DATE,
                        false,
                        true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata",
                        "commitBranch",
                        null,
                        BuildConstants.GIT_BRANCH == null ? "null" : BuildConstants.GIT_BRANCH,
                        false,
                        true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata",
                        "gitDirty",
                        null,
                        switch (BuildConstants.DIRTY) {
                            case 0 -> "clean";
                            case 1 -> "dirty";
                            default -> "Unknown";
                        },
                        false,
                        true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata", "runtimeType", null, Robot.getRuntimeType().toString(), false, true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata",
                        "initDate",
                        null,
                        new SimpleDateFormat("dd MMM yyyy, hh:mm:ss a")
                                .format(Calendar.getInstance(TimeZone.getTimeZone("America/Detroit"))
                                        .getTime()),
                        false,
                        true)
                .close();
        makeStringWriterInitialEx(
                        "Metadata", "telemetryLevel", null, TelemetryConfig.telemetryLevel.name(), false, true)
                .close();

        try (StringWriter levelWriter = makeStringWriterEx("Telem", "telemetryLevel", null, false, true)) {
            levelWriter.set(TelemetryConfig.telemetryLevel.name());
        }

        println(TelemetryConfig.PREFIX + "Telemetry initialization complete");
        if (!TelemetryConfig.telemetryLevel.logToNT) {
            reportWarning(TelemetryConfig.PREFIX + "NT telemetry is disabled!", false);
        }
        if (!TelemetryConfig.telemetryLevel.logToFile) {
            reportWarning(TelemetryConfig.PREFIX + "Log telemetry is disabled!", false);
        }
        hasInited = true;
    }

    /**
     * Use instead of System.out.println
     * USE LIGHTLY! Do not use this in any periodic calls, as println can be very expensive
     */
    public static void println(String msg) {
        // if (msg == null) {
        //     msg = "null";
        // }

        // if (!TelemetryConfig.loggingLevel.logToFile) {
        //     System.out.println(msg);
        //     return;
        // }
        // DataLogManager.log(PREFIX + msg);
        System.out.println(msg);
    }

    /**
     * Report a warning and log it to the console
     */
    public static void reportWarning(String msg, boolean printTrace) {
        if (msg == null) msg = "null";

        StackTraceElement[] trace = Thread.currentThread().getStackTrace();
        if (printTrace) {
            try {
                trace = Arrays.copyOfRange(trace, 2, trace.length); // cut off the first two elements

                DriverStation.reportWarning(msg, trace);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg);
                e.printStackTrace();
            }
        } else {
            String locString;
            try {
                locString = trace[2].toString();

                DriverStationJNI.sendError(false, 1, false, msg, locString, "", true);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg);
                e.printStackTrace();
            }
        }
    }

    /**
     * Report a warning and log it to the console
     */
    public static void reportWarning(Throwable e, boolean printTrace) {
        if (e == null) {
            reportError("Attempted to report a null Throwable as a warning", true);
        } else {
            String msg = "(" + e.getClass().getTypeName() + ") " + e.getMessage();
            if (printTrace) {
                DriverStation.reportWarning(msg, e.getStackTrace());
            } else {
                DriverStationJNI.sendError(false, 1, false, msg, e.getStackTrace()[0].toString(), "", true);
            }
        }
    }

    /**
     * Report an error and log it to the console
     */
    public static void reportError(String msg, boolean printTrace) {
        if (msg == null) msg = "null";

        StackTraceElement[] trace = Thread.currentThread().getStackTrace();
        if (printTrace) {
            try {
                trace = Arrays.copyOfRange(trace, 2, trace.length); // cut off the first two elements

                DriverStation.reportError(msg, trace);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg);
                e.printStackTrace();
            }
        } else {
            String locString;
            try {
                locString = trace[2].toString();

                DriverStationJNI.sendError(true, 1, false, msg, locString, "", true);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg);
                e.printStackTrace();
            }
        }
    }

    /**
     * Report an error and log it to the console
     */
    public static void reportError(Throwable e, boolean printTrace) {
        if (e == null) {
            reportError("Attempted to report a null Throwable as an error", true);
        } else {
            String msg = "(" + e.getClass().getTypeName() + ") " + e.getMessage();
            if (printTrace) {
                DriverStation.reportError(msg, e.getStackTrace());
            } else {
                DriverStationJNI.sendError(true, 1, false, msg, e.getStackTrace()[0].toString(), "", true);
            }
        }
    }

    /*
    NOTE: NetworkTable.normalizeKey is very slow, however it is safe to use here because it is only being run once per entry initialization
    */

    /*
    boolean[]
    */
    public static BoolArrayWriter makeBoolArrayWriter(String table, String name) {
        return makeBoolArrayWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriter(String table, String name, String unit) {
        return makeBoolArrayWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriterInitial(String table, String name, boolean[] initialValue) {
        return makeBoolArrayWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriterInitial(
            String table, String name, String unit, boolean[] initialValue) {
        return makeBoolArrayWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new BoolArrayWriter(
                NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriterInitialEx(
            String table,
            String name,
            String unit,
            boolean[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeBoolArrayWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static BooleanArraySubscriber makeBoolArrayReader(String table, String name, boolean[] defaultValue) {
        return NetworkTableInstance.getDefault()
                .getBooleanArrayTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    boolean
    */
    public static BoolWriter makeBoolWriter(String table, String name) {
        return makeBoolWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static BoolWriter makeBoolWriter(String table, String name, String unit) {
        return makeBoolWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static BoolWriter makeBoolWriterInitial(String table, String name, boolean initialValue) {
        return makeBoolWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static BoolWriter makeBoolWriterInitial(String table, String name, String unit, boolean initialValue) {
        return makeBoolWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static BoolWriter makeBoolWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new BoolWriter(NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static BoolWriter makeBoolWriterInitialEx(
            String table,
            String name,
            String unit,
            boolean initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeBoolWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static BooleanSubscriber makeBoolReader(String table, String name, boolean defaultValue) {
        return NetworkTableInstance.getDefault()
                .getBooleanTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    double[]
    */
    public static DoubleArrayWriter makeDoubleArrayWriter(String table, String name) {
        return makeDoubleArrayWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriter(String table, String name, String unit) {
        return makeDoubleArrayWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriterInitial(String table, String name, double[] initialValue) {
        return makeDoubleArrayWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriterInitial(
            String table, String name, String unit, double[] initialValue) {
        return makeDoubleArrayWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new DoubleArrayWriter(
                NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriterInitialEx(
            String table,
            String name,
            String unit,
            double[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeDoubleArrayWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static DoubleArraySubscriber makeDoubleArrayReader(String table, String name, double[] defaultValue) {
        return NetworkTableInstance.getDefault()
                .getDoubleArrayTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    double
    */
    public static DoubleWriter makeDoubleWriter(String table, String name) {
        return makeDoubleWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleWriter makeDoubleWriter(String table, String name, String unit) {
        return makeDoubleWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleWriter makeDoubleWriterInitial(String table, String name, double initialValue) {
        return makeDoubleWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleWriter makeDoubleWriterInitial(String table, String name, String unit, double initialValue) {
        return makeDoubleWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleWriter makeDoubleWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new DoubleWriter(NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static DoubleWriter makeDoubleWriterInitialEx(
            String table,
            String name,
            String unit,
            double initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeDoubleWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static DoubleSubscriber makeDoubleReader(String table, String name, double defaultValue) {
        return NetworkTableInstance.getDefault()
                .getDoubleTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    long[]
    */
    public static LongArrayWriter makeLongArrayWriter(String table, String name) {
        return makeLongArrayWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static LongArrayWriter makeLongArrayWriter(String table, String name, String unit) {
        return makeLongArrayWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static LongArrayWriter makeLongArrayWriterInitial(String table, String name, long[] initialValue) {
        return makeLongArrayWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static LongArrayWriter makeLongArrayWriterInitial(
            String table, String name, String unit, long[] initialValue) {
        return makeLongArrayWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static LongArrayWriter makeLongArrayWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new LongArrayWriter(
                NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static LongArrayWriter makeLongArrayWriterInitialEx(
            String table,
            String name,
            String unit,
            long[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeLongArrayWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static IntegerArraySubscriber makeLongArrayReader(String table, String name, long[] defaultValue) {
        return NetworkTableInstance.getDefault()
                .getIntegerArrayTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    long
    */
    public static LongWriter makeLongWriter(String table, String name) {
        return makeLongWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static LongWriter makeLongWriter(String table, String name, String unit) {
        return makeLongWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static LongWriter makeLongWriterInitial(String table, String name, long initialValue) {
        return makeLongWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static LongWriter makeLongWriterInitial(String table, String name, String unit, long initialValue) {
        return makeLongWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static LongWriter makeLongWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new LongWriter(NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static LongWriter makeLongWriterInitialEx(
            String table,
            String name,
            String unit,
            long initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeLongWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static IntegerSubscriber makeLongReader(String table, String name, long defaultValue) {
        return NetworkTableInstance.getDefault()
                .getIntegerTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    byte[]
    */
    public static RawWriter makeRawWriter(String typeString, String table, String name) {
        return makeRawWriterEx(
                typeString,
                table,
                name,
                null,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static RawWriter makeRawWriter(String typeString, String table, String name, String unit) {
        return makeRawWriterEx(
                typeString,
                table,
                name,
                unit,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static RawWriter makeRawWriterInitial(String typeString, String table, String name, byte[] initialValue) {
        return makeRawWriterInitialEx(
                typeString,
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static RawWriter makeRawWriterInitial(
            String typeString, String table, String name, String unit, byte[] initialValue) {
        return makeRawWriterInitialEx(
                typeString,
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static RawWriter makeRawWriterEx(
            String typeString,
            String table,
            String name,
            String unit,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return new RawWriter(
                typeString, NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static RawWriter makeRawWriterInitialEx(
            String typeString,
            String table,
            String name,
            String unit,
            byte[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeRawWriterEx(typeString, table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static RawSubscriber makeRawReader(String typeString, String table, String name, byte[] defaultValue) {
        return NetworkTableInstance.getDefault()
                .getRawTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(typeString, defaultValue);
    }

    /*
    String[]
    */
    public static StringArrayWriter makeStringArrayWriter(String table, String name) {
        return makeStringArrayWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static StringArrayWriter makeStringArrayWriter(String table, String name, String unit) {
        return makeStringArrayWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static StringArrayWriter makeStringArrayWriterInitial(String table, String name, String[] initialValue) {
        return makeStringArrayWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static StringArrayWriter makeStringArrayWriterInitial(
            String table, String name, String unit, String[] initialValue) {
        return makeStringArrayWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static StringArrayWriter makeStringArrayWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new StringArrayWriter(
                NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static StringArrayWriter makeStringArrayWriterInitialEx(
            String table,
            String name,
            String unit,
            String[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeStringArrayWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static StringArraySubscriber makeStringArrayReader(String table, String name, String[] defaultValue) {
        return NetworkTableInstance.getDefault()
                .getStringArrayTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    String
    */
    public static StringWriter makeStringWriter(String table, String name) {
        return makeStringWriterEx(
                table, name, null, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static StringWriter makeStringWriter(String table, String name, String unit) {
        return makeStringWriterEx(
                table, name, unit, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static StringWriter makeStringWriterInitial(String table, String name, String initialValue) {
        return makeStringWriterInitialEx(
                table,
                name,
                null,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static StringWriter makeStringWriterInitial(String table, String name, String unit, String initialValue) {
        return makeStringWriterInitialEx(
                table,
                name,
                unit,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static StringWriter makeStringWriterEx(
            String table, String name, String unit, boolean includeNTInChecks, boolean disableChecks) {
        return new StringWriter(NetworkTable.normalizeKey(table + "/" + name), unit, includeNTInChecks, disableChecks);
    }

    public static StringWriter makeStringWriterInitialEx(
            String table,
            String name,
            String unit,
            String initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = makeStringWriterEx(table, name, unit, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    public static StringSubscriber makeStringReader(String table, String name, String defaultValue) {
        return NetworkTableInstance.getDefault()
                .getStringTopic(NetworkTable.normalizeKey(table + "/" + name))
                .subscribe(defaultValue);
    }

    /*
    Struct[]
    */
    private static <T extends StructSerializable, S extends Struct<T>> StructArrayWriter<T> makeStructArrayWriter(
            S struct,
            String table,
            String name,
            EqualityTest<T[]> isEqual,
            CloneOperation<T[]> clone,
            T[] nullFallback) {
        return makeStructArrayWriterEx(
                struct,
                table,
                name,
                isEqual,
                clone,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks,
                nullFallback);
    }

    private static <T extends StructSerializable, S extends Struct<T>>
            StructArrayWriter<T> makeStructArrayWriterInitial(
                    S struct,
                    String table,
                    String name,
                    T[] initialValue,
                    EqualityTest<T[]> isEqual,
                    CloneOperation<T[]> clone,
                    T[] nullFallback) {
        return makeStructArrayWriterInitialEx(
                struct,
                table,
                name,
                initialValue,
                isEqual,
                clone,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks,
                nullFallback);
    }

    private static <T extends StructSerializable, S extends Struct<T>> StructArrayWriter<T> makeStructArrayWriterEx(
            S struct,
            String table,
            String name,
            EqualityTest<T[]> isEqual,
            CloneOperation<T[]> clone,
            boolean includeNTInChecks,
            boolean disableChecks,
            T[] nullFallback) {
        return new StructArrayWriter<T>(
                struct,
                NetworkTable.normalizeKey(table + "/" + name),
                isEqual,
                clone,
                includeNTInChecks,
                disableChecks,
                nullFallback);
    }

    private static <T extends StructSerializable, S extends Struct<T>>
            StructArrayWriter<T> makeStructArrayWriterInitialEx(
                    S struct,
                    String table,
                    String name,
                    T[] initialValue,
                    EqualityTest<T[]> isEqual,
                    CloneOperation<T[]> clone,
                    boolean includeNTInChecks,
                    boolean disableChecks,
                    T[] nullFallback) {
        StructArrayWriter<T> entry = makeStructArrayWriterEx(
                struct, table, name, isEqual, clone, includeNTInChecks, disableChecks, nullFallback);
        entry.set(initialValue);
        return entry;
    }

    public static StructArrayWriter<Pose2d> makePose2dArrayWriter(String table, String name) {
        return makeStructArrayWriter(
                Pose2d.struct,
                table,
                name,
                EqualityTest.pose2dArrayEqualityTest,
                CloneOperation.pose2dArrayCloneOperation,
                pose2dArrayNullFallback);
    }

    public static StructArrayWriter<Pose2d> makePose2dArrayWriterInitial(
            String table, String name, Pose2d[] initialValue) {
        return makeStructArrayWriterInitial(
                Pose2d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose2dArrayEqualityTest,
                CloneOperation.pose2dArrayCloneOperation,
                pose2dArrayNullFallback);
    }

    public static StructArrayWriter<Pose2d> makePose2dArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterEx(
                Pose2d.struct,
                table,
                name,
                EqualityTest.pose2dArrayEqualityTest,
                CloneOperation.pose2dArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                pose2dArrayNullFallback);
    }

    public static StructArrayWriter<Pose2d> makePose2dArrayWriterInitialEx(
            String table, String name, Pose2d[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterInitialEx(
                Pose2d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose2dArrayEqualityTest,
                CloneOperation.pose2dArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                pose2dArrayNullFallback);
    }

    public static StructArrayWriter<Pose3d> makePose3dArrayWriter(String table, String name) {
        return makeStructArrayWriter(
                Pose3d.struct,
                table,
                name,
                EqualityTest.pose3dArrayEqualityTest,
                CloneOperation.pose3dArrayCloneOperation,
                pose3dArrayNullFallback);
    }

    public static StructArrayWriter<Pose3d> makePose3dArrayWriterInitial(
            String table, String name, Pose3d[] initialValue) {
        return makeStructArrayWriterInitial(
                Pose3d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose3dArrayEqualityTest,
                CloneOperation.pose3dArrayCloneOperation,
                pose3dArrayNullFallback);
    }

    public static StructArrayWriter<Pose3d> makePose3dArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterEx(
                Pose3d.struct,
                table,
                name,
                EqualityTest.pose3dArrayEqualityTest,
                CloneOperation.pose3dArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                pose3dArrayNullFallback);
    }

    public static StructArrayWriter<Pose3d> makePose3dArrayWriterInitialEx(
            String table, String name, Pose3d[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterInitialEx(
                Pose3d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose3dArrayEqualityTest,
                CloneOperation.pose3dArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                pose3dArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModuleState> makeSwerveModuleStateArrayWriter(String table, String name) {
        return makeStructArrayWriter(
                SwerveModuleState.struct,
                table,
                name,
                EqualityTest.swerveModuleStateArrayEqualityTest,
                CloneOperation.swerveModuleStateArrayCloneOperation,
                swerveModuleStateArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModuleState> makeSwerveModuleStateArrayWriterInitial(
            String table, String name, SwerveModuleState[] initialValue) {
        return makeStructArrayWriterInitial(
                SwerveModuleState.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModuleStateArrayEqualityTest,
                CloneOperation.swerveModuleStateArrayCloneOperation,
                swerveModuleStateArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModuleState> makeSwerveModuleStateArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterEx(
                SwerveModuleState.struct,
                table,
                name,
                EqualityTest.swerveModuleStateArrayEqualityTest,
                CloneOperation.swerveModuleStateArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModuleStateArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModuleState> makeSwerveModuleStateArrayWriterInitialEx(
            String table,
            String name,
            SwerveModuleState[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructArrayWriterInitialEx(
                SwerveModuleState.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModuleStateArrayEqualityTest,
                CloneOperation.swerveModuleStateArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModuleStateArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModulePosition> makeSwerveModulePositionArrayWriter(
            String table, String name) {
        return makeStructArrayWriter(
                SwerveModulePosition.struct,
                table,
                name,
                EqualityTest.swerveModulePositionArrayEqualityTest,
                CloneOperation.swerveModulePositionArrayCloneOperation,
                swerveModulePositionArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModulePosition> makeSwerveModulePositionArrayWriterInitial(
            String table, String name, SwerveModulePosition[] initialValue) {
        return makeStructArrayWriterInitial(
                SwerveModulePosition.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModulePositionArrayEqualityTest,
                CloneOperation.swerveModulePositionArrayCloneOperation,
                swerveModulePositionArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModulePosition> makeSwerveModulePositionArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterEx(
                SwerveModulePosition.struct,
                table,
                name,
                EqualityTest.swerveModulePositionArrayEqualityTest,
                CloneOperation.swerveModulePositionArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModulePositionArrayNullFallback);
    }

    public static StructArrayWriter<SwerveModulePosition> makeSwerveModulePositionArrayWriterInitialEx(
            String table,
            String name,
            SwerveModulePosition[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructArrayWriterInitialEx(
                SwerveModulePosition.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModulePositionArrayEqualityTest,
                CloneOperation.swerveModulePositionArrayCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModulePositionArrayNullFallback);
    }

    /*
    Struct
    */
    private static <T extends StructSerializable, S extends Struct<T>> StructWriter<T> makeStructWriter(
            S struct, String table, String name, EqualityTest<T> isEqual, CloneOperation<T> clone, T nullFallback) {
        return makeStructWriterEx(
                struct,
                table,
                name,
                isEqual,
                clone,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks,
                nullFallback);
    }

    private static <T extends StructSerializable, S extends Struct<T>> StructWriter<T> makeStructWriterInitial(
            S struct,
            String table,
            String name,
            T initialValue,
            EqualityTest<T> isEqual,
            CloneOperation<T> clone,
            T nullFallback) {
        return makeStructWriterInitialEx(
                struct,
                table,
                name,
                initialValue,
                isEqual,
                clone,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks,
                nullFallback);
    }

    private static <T extends StructSerializable, S extends Struct<T>> StructWriter<T> makeStructWriterEx(
            S struct,
            String table,
            String name,
            EqualityTest<T> isEqual,
            CloneOperation<T> clone,
            boolean includeNTInChecks,
            boolean disableChecks,
            T nullFallback) {
        return new StructWriter<T>(
                struct,
                NetworkTable.normalizeKey(table + "/" + name),
                isEqual,
                clone,
                includeNTInChecks,
                disableChecks,
                nullFallback);
    }

    private static <T extends StructSerializable, S extends Struct<T>> StructWriter<T> makeStructWriterInitialEx(
            S struct,
            String table,
            String name,
            T initialValue,
            EqualityTest<T> isEqual,
            CloneOperation<T> clone,
            boolean includeNTInChecks,
            boolean disableChecks,
            T nullFallback) {
        StructWriter<T> entry =
                makeStructWriterEx(struct, table, name, isEqual, clone, includeNTInChecks, disableChecks, nullFallback);
        entry.set(initialValue);
        return entry;
    }

    public static StructWriter<Pose2d> makePose2dWriter(String table, String name) {
        return makeStructWriter(
                Pose2d.struct,
                table,
                name,
                EqualityTest.pose2dEqualityTest,
                CloneOperation.pose2dCloneOperation,
                Pose2d.kZero);
    }

    public static StructWriter<Pose2d> makePose2dWriterInitial(String table, String name, Pose2d initialValue) {
        return makeStructWriterInitial(
                Pose2d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose2dEqualityTest,
                CloneOperation.pose2dCloneOperation,
                Pose2d.kZero);
    }

    public static StructWriter<Pose2d> makePose2dWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
                Pose2d.struct,
                table,
                name,
                EqualityTest.pose2dEqualityTest,
                CloneOperation.pose2dCloneOperation,
                includeNTInChecks,
                disableChecks,
                Pose2d.kZero);
    }

    public static StructWriter<Pose2d> makePose2dWriterInitialEx(
            String table, String name, Pose2d initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterInitialEx(
                Pose2d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose2dEqualityTest,
                CloneOperation.pose2dCloneOperation,
                includeNTInChecks,
                disableChecks,
                Pose2d.kZero);
    }

    public static StructWriter<Pose3d> makePose3dWriter(String table, String name) {
        return makeStructWriter(
                Pose3d.struct,
                table,
                name,
                EqualityTest.pose3dEqualityTest,
                CloneOperation.pose3dCloneOperation,
                Pose3d.kZero);
    }

    public static StructWriter<Pose3d> makePose3dWriterInitial(String table, String name, Pose3d initialValue) {
        return makeStructWriterInitial(
                Pose3d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose3dEqualityTest,
                CloneOperation.pose3dCloneOperation,
                Pose3d.kZero);
    }

    public static StructWriter<Pose3d> makePose3dWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
                Pose3d.struct,
                table,
                name,
                EqualityTest.pose3dEqualityTest,
                CloneOperation.pose3dCloneOperation,
                includeNTInChecks,
                disableChecks,
                Pose3d.kZero);
    }

    public static StructWriter<Pose3d> makePose3dWriterInitialEx(
            String table, String name, Pose3d initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterInitialEx(
                Pose3d.struct,
                table,
                name,
                initialValue,
                EqualityTest.pose3dEqualityTest,
                CloneOperation.pose3dCloneOperation,
                includeNTInChecks,
                disableChecks,
                Pose3d.kZero);
    }

    public static StructWriter<ChassisSpeeds> makeChassisSpeedsWriter(String table, String name) {
        return makeStructWriter(
                ChassisSpeeds.struct,
                table,
                name,
                EqualityTest.chassisSpeedsEqualityTest,
                CloneOperation.chassisSpeedsCloneOperation,
                chassisSpeedsNullFallback);
    }

    public static StructWriter<ChassisSpeeds> makeChassisSpeedsWriterInitial(
            String table, String name, ChassisSpeeds initialValue) {
        return makeStructWriterInitial(
                ChassisSpeeds.struct,
                table,
                name,
                initialValue,
                EqualityTest.chassisSpeedsEqualityTest,
                CloneOperation.chassisSpeedsCloneOperation,
                chassisSpeedsNullFallback);
    }

    public static StructWriter<ChassisSpeeds> makeChassisSpeedsWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
                ChassisSpeeds.struct,
                table,
                name,
                EqualityTest.chassisSpeedsEqualityTest,
                CloneOperation.chassisSpeedsCloneOperation,
                includeNTInChecks,
                disableChecks,
                chassisSpeedsNullFallback);
    }

    public static StructWriter<ChassisSpeeds> makeChassisSpeedsWriterInitialEx(
            String table, String name, ChassisSpeeds initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterInitialEx(
                ChassisSpeeds.struct,
                table,
                name,
                initialValue,
                EqualityTest.chassisSpeedsEqualityTest,
                CloneOperation.chassisSpeedsCloneOperation,
                includeNTInChecks,
                disableChecks,
                chassisSpeedsNullFallback);
    }

    public static StructWriter<SwerveModuleState> makeSwerveModuleStateWriter(String table, String name) {
        return makeStructWriter(
                SwerveModuleState.struct,
                table,
                name,
                EqualityTest.swerveModuleStateEqualityTest,
                CloneOperation.swerveModuleStateCloneOperation,
                swerveModuleStateNullFallback);
    }

    public static StructWriter<SwerveModuleState> makeSwerveModuleStateWriterInitial(
            String table, String name, SwerveModuleState initialValue) {
        return makeStructWriterInitial(
                SwerveModuleState.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModuleStateEqualityTest,
                CloneOperation.swerveModuleStateCloneOperation,
                swerveModuleStateNullFallback);
    }

    public static StructWriter<SwerveModuleState> makeSwerveModuleStateWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
                SwerveModuleState.struct,
                table,
                name,
                EqualityTest.swerveModuleStateEqualityTest,
                CloneOperation.swerveModuleStateCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModuleStateNullFallback);
    }

    public static StructWriter<SwerveModuleState> makeSwerveModuleStateWriterInitialEx(
            String table,
            String name,
            SwerveModuleState initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructWriterInitialEx(
                SwerveModuleState.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModuleStateEqualityTest,
                CloneOperation.swerveModuleStateCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModuleStateNullFallback);
    }

    public static StructWriter<SwerveModulePosition> makeSwerveModulePositionWriter(String table, String name) {
        return makeStructWriter(
                SwerveModulePosition.struct,
                table,
                name,
                EqualityTest.swerveModulePositionEqualityTest,
                CloneOperation.swerveModulePositionCloneOperation,
                swerveModulePositionNullFallback);
    }

    public static StructWriter<SwerveModulePosition> makeSwerveModulePositionWriterInitial(
            String table, String name, SwerveModulePosition initialValue) {
        return makeStructWriterInitial(
                SwerveModulePosition.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModulePositionEqualityTest,
                CloneOperation.swerveModulePositionCloneOperation,
                swerveModulePositionNullFallback);
    }

    public static StructWriter<SwerveModulePosition> makeSwerveModulePositionWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
                SwerveModulePosition.struct,
                table,
                name,
                EqualityTest.swerveModulePositionEqualityTest,
                CloneOperation.swerveModulePositionCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModulePositionNullFallback);
    }

    public static StructWriter<SwerveModulePosition> makeSwerveModulePositionWriterInitialEx(
            String table,
            String name,
            SwerveModulePosition initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructWriterInitialEx(
                SwerveModulePosition.struct,
                table,
                name,
                initialValue,
                EqualityTest.swerveModulePositionEqualityTest,
                CloneOperation.swerveModulePositionCloneOperation,
                includeNTInChecks,
                disableChecks,
                swerveModulePositionNullFallback);
    }

    /*
    Subsystem
    */
    public static <T extends Subsystem> SubsystemWriter<T> makeSubsystemWriter(T subsystem, String table) {
        return new SubsystemWriter<T>(subsystem, NetworkTable.normalizeKey(table + "/" + subsystem.getName()));
    }

    /*
    RobotState
    */
    public static RobotStateWriter makeRobotStateWriter(String table, String name) {
        return new RobotStateWriter(NetworkTable.normalizeKey(table + "/" + name), robotStateNullFallback);
    }

    public static RobotStateWriter makeRobotStateWriterInitial(String table, String name, RobotState initialValue) {
        var entry = makeRobotStateWriter(table, name);
        entry.set(initialValue);
        return entry;
    }

    /*
    LauncherReport
    */
    public static LauncherReportWriter makeLauncherReportWriter(String table, String name) {
        return new LauncherReportWriter(NetworkTable.normalizeKey(table + "/" + name), launcherReportNullFallback);
    }

    public static LauncherReportWriter makeLauncherReportWriterInitial(
            String table, String name, LauncherReport initialValue) {
        var entry = makeLauncherReportWriter(table, name);
        entry.set(initialValue);
        return entry;
    }

    /*
    TurretAngle
    */
    public static TurretAngleWriter makeTurretAngleWriter(String table, String name) {
        return makeTurretAngleWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static TurretAngleWriter makeTurretAngleWriterInitial(String table, String name, TurretAngle initialValue) {
        return makeTurretAngleWriterInitialEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static TurretAngleWriter makeTurretAngleWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new TurretAngleWriter(
                NetworkTable.normalizeKey(table + "/" + name),
                includeNTInChecks,
                disableChecks,
                turretAngleNullFallback);
    }

    public static TurretAngleWriter makeTurretAngleWriterInitialEx(
            String table, String name, TurretAngle initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry = makeTurretAngleWriterEx(table, name, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    ShooterTarget
    */
    public static ShooterTargetWriter makeShooterTargetWriter(String table, String name) {
        return makeShooterTargetWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static ShooterTargetWriter makeShooterTargetWriterInitial(
            String table, String name, ShooterTarget initialValue) {
        return makeShooterTargetWriterInitialEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static ShooterTargetWriter makeShooterTargetWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new ShooterTargetWriter(
                NetworkTable.normalizeKey(table + "/" + name),
                includeNTInChecks,
                disableChecks,
                shooterTargetAngleNullFallback);
    }

    public static ShooterTargetWriter makeShooterTargetWriterInitialEx(
            String table, String name, ShooterTarget initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry = makeShooterTargetWriterEx(table, name, includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    private Telemetry() {}
}
