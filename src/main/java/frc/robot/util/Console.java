package frc.robot.util;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;

public final class Console {
    public static void println(String msg_nl) {
        System.out.println(msg_nl);
    }

    public static void reportCANDisconnect(String name, int canID, int channelID) {
        reportWarning(String.format("Lost connection to CAN %d (%s, ch %d)", canID, name, channelID), false);
    }

    public static void reportCANDisconnectNoChannel(String name, int canID) {
        reportWarning(String.format("Lost connection to CAN %d (%s)", canID, name), false);
    }

    public static void reportCANConnect(String name, int canID, int channelID) {
        println(String.format("Connected to CAN %d (%s, ch %d)", canID, name, channelID));
    }

    public static void reportCANConnectNoChannel(String name, int canID) {
        println(String.format("Connected to CAN %d (%s)", canID, name));
    }

    public static void reportBreakerTrip(String name, int canID, int channelID) {
        reportWarning(String.format("Breaker ch %d (%s, CAN %d) tripped", channelID, name, canID), false);
    }

    public static void reportBreakerTripNoCAN(String name, int channelID) {
        reportWarning(String.format("Breaker ch %d (%s) tripped", channelID, name), false);
    }

    public static void reportBreakerReset(String name, int canID, int channelID) {
        println(String.format("Breaker ch %d (%s, CAN %d) reset", channelID, name, canID));
    }

    public static void reportBreakerResetNoCAN(String name, int channelID) {
        println(String.format("Breaker ch %d (%s, CAN %d) reset", channelID, name));
    }

    public static void reportThermalShutdownTrigger(String name, int canID, int channelID) {
        reportWarning(String.format("Thermal shutdown triggered on %s (CAN %d, ch %d)", name, canID, channelID), false);
    }

    public static void reportThermalShutdownRelease(String name, int canID, int channelID) {
        println(String.format("Thermal shutdown released on %s (CAN %d, ch %d)", name, canID, channelID));
    }

    public static void reportControllerDisconnect(String name, int port) {
        reportWarning(String.format("Lost connection to controller %s (port %d)", name, port), false);
    }

    public static void reportControllerConnect(String name, int port) {
        println(String.format("Connected to controller %s (port %d)", name, port));
    }

    public static void reportWarning(String msg_nl, boolean printTrace) {
        if (msg_nl == null) msg_nl = "null";

        StackTraceElement[] trace = Thread.currentThread().getStackTrace();
        if (printTrace) {
            try {
                trace = Arrays.copyOfRange(trace, 2, trace.length); // cut off the first two elements

                DriverStation.reportWarning(msg_nl, trace);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg_nl);
                e.printStackTrace();
            }
        } else {
            String locString;
            try {
                locString = trace[2].toString();

                DriverStationJNI.sendError(false, 1, false, msg_nl, locString, "", true);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg_nl);
                e.printStackTrace();
            }
        }
    }

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

    public static void reportError(String msg_nl, boolean printTrace) {
        if (msg_nl == null) msg_nl = "null";

        StackTraceElement[] trace = Thread.currentThread().getStackTrace();
        if (printTrace) {
            try {
                trace = Arrays.copyOfRange(trace, 2, trace.length); // cut off the first two elements

                DriverStation.reportError(msg_nl, trace);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg_nl);
                e.printStackTrace();
            }
        } else {
            String locString;
            try {
                locString = trace[2].toString();

                DriverStationJNI.sendError(true, 1, false, msg_nl, locString, "", true);
            } catch (Exception e) {
                println("Exception occurred while reporting warning with message: " + msg_nl);
                e.printStackTrace();
            }
        }
    }

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

    private Console() {}
}
