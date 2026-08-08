package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.telemetry.Telemetry;
import java.util.ArrayList;
import java.util.List;

public class ControllerUtil {

    private static class Rumble {
        public double lStrength, rStrength, seconds;
        public Timer timer;

        public Rumble(double l, double r, double sec) {
            lStrength = l;
            rStrength = r;
            seconds = sec;

            timer = new Timer();
            timer.start();
        }
    }

    public static final int MAX_RUMBLES = 10;

    // can't have array of lists, have to use list of lists
    private static List<List<Rumble>> rumbles;
    private static List<Pair<Double, Double>> lastOutputs;

    static {
        rumbles = new ArrayList<List<Rumble>>();
        rumbles.add(0, new ArrayList<Rumble>());
        rumbles.add(1, new ArrayList<Rumble>());
        rumbles.add(2, new ArrayList<Rumble>());
        rumbles.add(3, new ArrayList<Rumble>());
        rumbles.add(4, new ArrayList<Rumble>());
        rumbles.add(5, new ArrayList<Rumble>());

        lastOutputs = new ArrayList<Pair<Double, Double>>();
        lastOutputs.add(0, new Pair<Double, Double>(0.0, 0.0));
        lastOutputs.add(1, new Pair<Double, Double>(0.0, 0.0));
        lastOutputs.add(2, new Pair<Double, Double>(0.0, 0.0));
        lastOutputs.add(3, new Pair<Double, Double>(0.0, 0.0));
        lastOutputs.add(4, new Pair<Double, Double>(0.0, 0.0));
        lastOutputs.add(5, new Pair<Double, Double>(0.0, 0.0));
    }

    // prevent instantiating objects
    private ControllerUtil() {}

    public static void periodic(GenericHID... controllers) {
        // rather dumb system, but it works
        for (GenericHID contr : controllers) {
            if (contr == null) continue; // silently continue

            int contrId = contr.getPort();
            if (contrId < 0 || contrId > 5) {
                Telemetry.reportWarning("Controller list contains controller with invalid port", true);
                continue;
            }

            // calc the sum of all of the rumbles scheduled for this controller.
            // if there are none, then leftSum and rightSum will be left as 0, stopping the rumble
            List<Rumble> contrRumbles = rumbles.get(contrId);
            double leftSum = 0, rightSum = 0;
            for (int i = 0; i < contrRumbles.size(); i++) {
                Rumble rumble = contrRumbles.get(i);

                // check if rumble has expired
                if (rumble.timer.hasElapsed(rumble.seconds)) {
                    contrRumbles.remove(i);
                    i--; // account for decrease in length of list
                    continue;
                }

                leftSum += rumble.lStrength;
                rightSum += rumble.rStrength;
            }

            // dumb cap
            leftSum = Math.min(Math.max(leftSum, 0), 1);
            rightSum = Math.min(Math.max(rightSum, 0), 1);

            Pair<Double, Double> prevOut = lastOutputs.get(contrId);

            if (prevOut.getFirst() != leftSum) {
                contr.setRumble(RumbleType.kLeftRumble, leftSum);
            }
            if (prevOut.getSecond() != rightSum) {
                contr.setRumble(RumbleType.kRightRumble, rightSum);
            }
            lastOutputs.set(contrId, new Pair<Double, Double>(leftSum, rightSum));
        }
    }

    public static boolean scheduleControllerRumble(int id, double lStrength, double rStrength, double seconds) {
        if (id < 0 || id > 5) {
            Telemetry.reportError("Invalid controller id", true);
            return false;
        }
        if (seconds <= 0) {
            Telemetry.reportWarning("Cannot schedule a rumble with a length <= 0", true);
            return false;
        }
        if (getActiveRumbleCount(id) >= MAX_RUMBLES) {
            Telemetry.reportWarning("Max rumble limit hit on controller " + id + ", cancelling requested rumble", true);
            return false;
        }

        lStrength = MathUtil.clamp(lStrength, 0, 1);
        rStrength = MathUtil.clamp(rStrength, 0, 1);
        if (lStrength == 0 && rStrength == 0) {
            return false; // quietly return, nothing to be done
        }

        rumbles.get(id).add(new Rumble(lStrength, rStrength, seconds));
        return true;
    }

    public static void cancelControllerRumbles(int id) {
        if (id < 0 || id > 5) {
            Telemetry.reportError("Invalid controller id", true);
            return;
        }

        rumbles.get(id).clear();
    }

    public static int getActiveRumbleCount(int id) {
        if (id < 0 || id > 5) {
            Telemetry.reportError("Invalid controller id", true);
            return 0;
        }

        return rumbles.get(id).size();
    }

    /*
     * See deadband graphs here: https://www.desmos.com/calculator/994aac3787
     */

    public static double applySimpleDeadband(double raw, double deadband) {
        return Math.abs(raw) <= deadband ? 0 : raw;
    }

    public static double applyLinearDeadband(double raw, double deadband) {
        return Math.abs(raw) <= deadband ? 0 : ((raw - deadband * Math.signum(raw)) / (1 - deadband));
    }

    public static double applyExponentialDeadband(double raw, double deadband, int power) {
        return Math.pow(Math.abs(applyLinearDeadband(raw, deadband)), power) * Math.signum(raw);
    }
}
