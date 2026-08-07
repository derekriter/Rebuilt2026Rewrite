package frc.robot.telemetry;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.config.TelemetryConfig;
import frc.robot.telemetry.writer.BoolArrayWriter;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleArrayWriter;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.telemetry.writer.FloatArrayWriter;
import frc.robot.telemetry.writer.FloatWriter;
import frc.robot.telemetry.writer.LongArrayWriter;
import frc.robot.telemetry.writer.LongWriter;
import frc.robot.telemetry.writer.RawWriter;
import frc.robot.telemetry.writer.StringArrayWriter;
import frc.robot.telemetry.writer.StringWriter;
import frc.robot.telemetry.writer.StructArrayWriter;
import frc.robot.telemetry.writer.StructWriter;
import frc.robot.util.CloneOperation;
import frc.robot.util.EqualityTest;
import java.lang.reflect.Field;

public class Telemetry {

    // TODO: reader classes
    // TODO: review messaging
    // TODO: add log metadata

    private static final Alert flashdriveNotRecognizedAlert =
            new Alert("Logging flashdrive not recognized", AlertType.kError);

    private static boolean hasInited = false;

    private static final Pose2d[] pose2dArrayNullFallback = new Pose2d[0];
    private static final Pose3d[] pose3dArrayNullFallback = new Pose3d[0];
    private static final ChassisSpeeds chassisSpeedsNullFallback = new ChassisSpeeds();
    private static final SwerveModuleState swerveModuleStateNullFallback = new SwerveModuleState();
    private static final SwerveModuleState[] swerveModuleStateArrayNullFallback = new SwerveModuleState[0];
    private static final SwerveModulePosition swerveModulePositionNullFallback = new SwerveModulePosition();
    private static final SwerveModulePosition[] swerveModulePositionArrayNullFallback = new SwerveModulePosition[0];

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

            try (StringWriter directoryWriter = makeStringWriter("Telem", "logDirectory");
                    BoolWriter recognizedWriter = makeBoolWriter("Telem", "flashdriveRecognized")) {
                String logDir = DataLogManager.getLogDir();
                directoryWriter.set(logDir);

                boolean loggingToFlash =
                        logDir == null ? false : logDir.toLowerCase().startsWith("/u");
                recognizedWriter.set(loggingToFlash);
                flashdriveNotRecognizedAlert.set(!loggingToFlash);
            }
        }

        if (TelemetryConfig.telemetryLevel.logToNT || TelemetryConfig.telemetryLevel.logToFile) {
            println(TelemetryConfig.PREFIX + "Logging started");
        } else {
            println(TelemetryConfig.PREFIX + "Logging initialization done, logging is disabled");
        }
        hasInited = true;
    }

    /**
     * Use instead of System.out.println
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
    public static void reportWarning(String msg, boolean printFullTrace) {
        reportWarning(msg, generateStackTrace(Thread.currentThread().getStackTrace(), 2), printFullTrace);
    }

    /**
     * Report a warning and log it to the console
     */
    public static void reportWarning(Throwable e, boolean printFullTrace) {
        if (e == null) {
            reportWarning("Unknown warning exception, attempted to log null Throwable", true);
        } else {
            reportWarning(e.getMessage(), generateStackTrace(e.getStackTrace(), 0), printFullTrace);
        }
    }

    private static void reportWarning(String msg, StackTrace trace, boolean printFullTrace) {
        // safe because of short circuit logic evalutation
        if (msg == null || msg.isEmpty()) msg = "No message provided";
        if (trace == null) {
            trace = new StackTrace();
            trace.location = "Invalid StackTrace";
            trace.trace = "Invalid StackTrace";
        }
        if (trace.location == null) {
            trace.location = "Invalid StackTrace.location";
        }
        if (trace.trace == null) {
            trace.trace = "Invalid StackTrace.trace";
        }

        StringBuilder builder = new StringBuilder();
        builder.append("Warning at ");
        builder.append(trace.location);
        builder.append(": ");
        builder.append(msg);

        if (printFullTrace) {
            builder.append('\n');
            builder.append(trace.trace);
        }

        println(builder.toString());
    }

    /**
     * Report an error and log it to the console
     */
    public static void reportError(String msg) {
        reportError(msg, generateStackTrace(Thread.currentThread().getStackTrace(), 2));
    }

    /**
     * Report an error and log it to the console
     */
    public static void reportError(Throwable e) {
        if (e == null) {
            reportError("Unknown error exception, attempted to log null Throwable");
        } else {
            reportError(e.getMessage(), generateStackTrace(e.getStackTrace(), 0));
        }
    }

    private static void reportError(String msg, StackTrace trace) {
        if (msg == null || msg.isEmpty()) msg = "No message provided";
        if (trace == null) {
            trace = new StackTrace();
            trace.location = "Invalid StackTrace";
            trace.trace = "Invalid StackTrace";
        }
        if (trace.location == null) {
            trace.location = "Invalid StackTrace.location";
        }
        if (trace.trace == null) {
            trace.trace = "Invalid StackTrace.trace";
        }

        StringBuilder builder = new StringBuilder();
        builder.append("ERROR at ");
        builder.append(trace.location);
        builder.append(": ");
        builder.append(msg);
        builder.append('\n');
        builder.append(trace.trace);

        println(builder.toString());
    }

    private static class StackTrace {
        public String location;
        public String trace;
    }

    private static StackTrace generateStackTrace(StackTraceElement[] trace, int offset) {
        // stole this code from DriverStation.class:494
        String locString;
        if (trace.length >= offset + 1) {
            locString = trace[offset].toString();
        } else {
            locString = "";
        }

        StringBuilder traceString = new StringBuilder();
        boolean haveLoc = false;
        for (int i = offset; i < trace.length; i++) {
            String loc = trace[i].toString();
            traceString.append("\tat ").append(loc).append('\n');

            // get first user function
            if (!haveLoc && !loc.startsWith("edu.wpi.first")) {
                locString = loc;
                haveLoc = true;
            }
        }

        StackTrace result = new StackTrace();
        result.location = locString;
        result.trace = traceString.toString();
        return result;
    }

    /*
    NOTE: NetworkTable.normalizeKey is very slow, however it is safe to use here because it is only being run once per entry initialization
    */

    /*
    boolean[]
    */
    public static BoolArrayWriter makeBoolArrayWriter(String table, String name) {
        return makeBoolArrayWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriter(String table, String name, boolean[] initialValue) {
        return makeBoolArrayWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new BoolArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static BoolArrayWriter makeBoolArrayWriterEx(
            String table, String name, boolean[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry =
                new BoolArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    boolean
    */
    public static BoolWriter makeBoolWriter(String table, String name) {
        return makeBoolWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static BoolWriter makeBoolWriter(String table, String name, boolean initialValue) {
        return makeBoolWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static BoolWriter makeBoolWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new BoolWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static BoolWriter makeBoolWriterEx(
            String table, String name, boolean initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry = new BoolWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    double[]
    */
    public static DoubleArrayWriter makeDoubleArrayWriter(String table, String name) {
        return makeDoubleArrayWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriter(String table, String name, double[] initialValue) {
        return makeDoubleArrayWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new DoubleArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static DoubleArrayWriter makeDoubleArrayWriterEx(
            String table, String name, double[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry =
                new DoubleArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    double
    */
    public static DoubleWriter makeDoubleWriter(String table, String name) {
        return makeDoubleWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleWriter makeDoubleWriter(String table, String name, double initialValue) {
        return makeDoubleWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static DoubleWriter makeDoubleWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new DoubleWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static DoubleWriter makeDoubleWriterEx(
            String table, String name, double initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry = new DoubleWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    float[]
    */
    public static FloatArrayWriter makeFloatArrayWriter(String table, String name) {
        return makeFloatArrayWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static FloatArrayWriter makeFloatArrayWriter(String table, String name, float[] initialValue) {
        return makeFloatArrayWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static FloatArrayWriter makeFloatArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new FloatArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static FloatArrayWriter makeFloatArrayWriterEx(
            String table, String name, float[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry =
                new FloatArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    float
    */
    public static FloatWriter makeFloatWriter(String table, String name) {
        return makeFloatWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static FloatWriter makeFloatWriter(String table, String name, float initialValue) {
        return makeFloatWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static FloatWriter makeFloatWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new FloatWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static FloatWriter makeFloatWriterEx(
            String table, String name, float initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry = new FloatWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    long[]
    */
    public static LongArrayWriter makeLongArrayWriter(String table, String name) {
        return makeLongArrayWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static LongArrayWriter makeLongArrayWriter(String table, String name, long[] initialValue) {
        return makeLongArrayWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static LongArrayWriter makeLongArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new LongArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static LongArrayWriter makeLongArrayWriterEx(
            String table, String name, long[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry =
                new LongArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    long
    */
    public static LongWriter makeLongWriter(String table, String name) {
        return makeLongWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static LongWriter makeLongWriter(String table, String name, long initialValue) {
        return makeLongWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static LongWriter makeLongWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new LongWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static LongWriter makeLongWriterEx(
            String table, String name, long initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry = new LongWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    byte[]
    */
    public static RawWriter makeRawWriter(String typeString, String table, String name) {
        return makeRawWriterEx(
                typeString,
                table,
                name,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static RawWriter makeRawWriter(String typeString, String table, String name, byte[] initialValue) {
        return makeRawWriterEx(
                typeString,
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static RawWriter makeRawWriterEx(
            String typeString, String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new RawWriter(
                typeString, NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static RawWriter makeRawWriterEx(
            String typeString,
            String table,
            String name,
            byte[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        var entry = new RawWriter(
                typeString, NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    String[]
    */
    public static StringArrayWriter makeStringArrayWriter(String table, String name) {
        return makeStringArrayWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static StringArrayWriter makeStringArrayWriter(String table, String name, String[] initialValue) {
        return makeStringArrayWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static StringArrayWriter makeStringArrayWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new StringArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static StringArrayWriter makeStringArrayWriterEx(
            String table, String name, String[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry =
                new StringArrayWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
    }

    /*
    String
    */
    public static StringWriter makeStringWriter(String table, String name) {
        return makeStringWriterEx(
                table, name, TelemetryConfig.defaultIncludeNTInChecks, TelemetryConfig.defaultDisableChecks);
    }

    public static StringWriter makeStringWriter(String table, String name, String initialValue) {
        return makeStringWriterEx(
                table,
                name,
                initialValue,
                TelemetryConfig.defaultIncludeNTInChecks,
                TelemetryConfig.defaultDisableChecks);
    }

    public static StringWriter makeStringWriterEx(
            String table, String name, boolean includeNTInChecks, boolean disableChecks) {
        return new StringWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
    }

    public static StringWriter makeStringWriterEx(
            String table, String name, String initialValue, boolean includeNTInChecks, boolean disableChecks) {
        var entry = new StringWriter(NetworkTable.normalizeKey(table + "/" + name), includeNTInChecks, disableChecks);
        entry.set(initialValue);
        return entry;
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

    private static <T extends StructSerializable, S extends Struct<T>> StructArrayWriter<T> makeStructArrayWriter(
            S struct,
            String table,
            String name,
            T[] initialValue,
            EqualityTest<T[]> isEqual,
            CloneOperation<T[]> clone,
            T[] nullFallback) {
        return makeStructArrayWriterEx(
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

    private static <T extends StructSerializable, S extends Struct<T>> StructArrayWriter<T> makeStructArrayWriterEx(
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

    public static StructArrayWriter<Pose2d> makePose2dArrayWriter(String table, String name, Pose2d[] initialValue) {
        return makeStructArrayWriter(
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

    public static StructArrayWriter<Pose2d> makePose2dArrayWriterEx(
            String table, String name, Pose2d[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterEx(
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

    public static StructArrayWriter<Pose3d> makePose3dArrayWriter(String table, String name, Pose3d[] initialValue) {
        return makeStructArrayWriter(
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

    public static StructArrayWriter<Pose3d> makePose3dArrayWriterEx(
            String table, String name, Pose3d[] initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructArrayWriterEx(
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

    public static StructArrayWriter<SwerveModuleState> makeSwerveModuleStateArrayWriter(
            String table, String name, SwerveModuleState[] initialValue) {
        return makeStructArrayWriter(
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

    public static StructArrayWriter<SwerveModuleState> makeSwerveModuleStateArrayWriterEx(
            String table,
            String name,
            SwerveModuleState[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructArrayWriterEx(
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

    public static StructArrayWriter<SwerveModulePosition> makeSwerveModulePositionArrayWriter(
            String table, String name, SwerveModulePosition[] initialValue) {
        return makeStructArrayWriter(
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

    public static StructArrayWriter<SwerveModulePosition> makeSwerveModulePositionArrayWriterEx(
            String table,
            String name,
            SwerveModulePosition[] initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructArrayWriterEx(
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

    private static <T extends StructSerializable, S extends Struct<T>> StructWriter<T> makeStructWriter(
            S struct,
            String table,
            String name,
            T initialValue,
            EqualityTest<T> isEqual,
            CloneOperation<T> clone,
            T nullFallback) {
        return makeStructWriterEx(
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

    private static <T extends StructSerializable, S extends Struct<T>> StructWriter<T> makeStructWriterEx(
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

    public static StructWriter<Pose2d> makePose2dWriter(String table, String name, Pose2d initialValue) {
        return makeStructWriter(
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

    public static StructWriter<Pose2d> makePose2dWriterEx(
            String table, String name, Pose2d initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
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

    public static StructWriter<Pose3d> makePose3dWriter(String table, String name, Pose3d initialValue) {
        return makeStructWriter(
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

    public static StructWriter<Pose3d> makePose3dWriterEx(
            String table, String name, Pose3d initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
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

    public static StructWriter<ChassisSpeeds> makeChassisSpeedsWriter(
            String table, String name, ChassisSpeeds initialValue) {
        return makeStructWriter(
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

    public static StructWriter<ChassisSpeeds> makeChassisSpeedsWriterEx(
            String table, String name, ChassisSpeeds initialValue, boolean includeNTInChecks, boolean disableChecks) {
        return makeStructWriterEx(
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

    public static StructWriter<SwerveModuleState> makeSwerveModuleStateWriter(
            String table, String name, SwerveModuleState initialValue) {
        return makeStructWriter(
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

    public static StructWriter<SwerveModuleState> makeSwerveModuleStateWriterEx(
            String table,
            String name,
            SwerveModuleState initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructWriterEx(
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

    public static StructWriter<SwerveModulePosition> makeSwerveModulePositionWriter(
            String table, String name, SwerveModulePosition initialValue) {
        return makeStructWriter(
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

    public static StructWriter<SwerveModulePosition> makeSwerveModulePositionWriterEx(
            String table,
            String name,
            SwerveModulePosition initialValue,
            boolean includeNTInChecks,
            boolean disableChecks) {
        return makeStructWriterEx(
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

    private Telemetry() {}
}
