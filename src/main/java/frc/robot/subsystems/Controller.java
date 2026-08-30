package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Robot;
import frc.robot.commands.PulseRumble;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;

public class Controller<HID extends GenericHID, CMDHID extends CommandGenericHID> extends SubsystemBase {

    private final String name, uppercaseName;
    private final HID hid;
    private final CMDHID cmdHid_nl;

    private final Alert disconnAlert;

    private boolean connectedLast = false;

    public Controller(String _name, HID _hid, CMDHID _cmdHID_nl) {
        hid = _hid;
        cmdHid_nl = _cmdHID_nl;
        if (_name.isEmpty()) {
            name = String.format("unnamed%s_%d", hid.getClass().getSimpleName(), hid.getPort());
        } else {
            name = _name;
        }

        uppercaseName = Character.toUpperCase(name.charAt(0)) + name.substring(1);

        disconnAlert = new Alert(String.format("%s controller not connected", name), AlertType.kWarning);

        Logger.recordOutput("Controllers/" + uppercaseName + "/leftRumble", 0.0);
        Logger.recordOutput("Controllers/" + uppercaseName + "/rightRumble", 0.0);
    }

    @Override
    public void periodic() {
        boolean connected = hid.isConnected();

        Logger.recordOutput("Controllers/" + uppercaseName + "/connected", connected);
        disconnAlert.set(!connected);
        if (connected != connectedLast) {
            if (connected) {
                Console.reportControllerConnect(name, hid.getPort());
                CommandScheduler.getInstance().schedule(knockCmd(0.75));
            } else {
                Console.reportControllerDisconnect(name, hid.getPort());
            }
        }

        connectedLast = connected;

        Command currentCommand = getCurrentCommand();
        Logger.recordOutput(
                "Controllers/" + uppercaseName + "/currentCommand",
                currentCommand == null ? BlankValues.string : currentCommand.getName());

        Command defaultCommand = getDefaultCommand();
        Logger.recordOutput(
                "Controllers/" + uppercaseName + "/defaultCommand",
                defaultCommand == null ? BlankValues.string : defaultCommand.getName());
    }

    public HID getHID() {
        return hid;
    }

    public CMDHID getCommandHID_nl() {
        return cmdHid_nl;
    }

    public void setRumble(double left, double right) {
        left = MathUtil.clamp(left, 0, 1);
        right = MathUtil.clamp(right, 0, 1);

        hid.setRumble(RumbleType.kLeftRumble, left);
        hid.setRumble(RumbleType.kRightRumble, right);

        Logger.recordOutput("Controllers/" + uppercaseName + "/leftRumble", left);
        Logger.recordOutput("Controllers/" + uppercaseName + "/rightRumble", right);
    }

    public void stopRumble() {
        setRumble(0, 0);
    }

    public Command rumbleCmd(double strength) {
        return Commands.startEnd(() -> setRumble(strength, strength), this::stopRumble, this)
                .ignoringDisable(true)
                .withName("rumble");
    }

    public Command rumbleCmd(double strength, double length_s) {
        return rumbleCmd(strength).withTimeout(length_s).withName("rumble");
    }

    public Command pulseRumbleCmd(double onTime_s, double offTime_s, double strength) {
        return new PulseRumble(this, onTime_s, offTime_s, strength);
    }

    public Command pulseRumbleCmd(double onTime_s, double offTime_s, double strength, double length_s) {
        return pulseRumbleCmd(onTime_s, offTime_s, strength)
                .withTimeout(length_s)
                .withName("PulseRumble");
    }

    public Command knockCmd(double strength) {
        return pulseRumbleCmd(0.25, 0.1, strength, 0.35 * 2).withName("knock");
    }

    /*
     * See deadband graphs here: https://www.desmos.com/calculator/994aac3787
     */

    public static boolean isPastDeadband(double raw, double deadband) {
        return raw < -deadband || raw > deadband;
    }

    public static boolean isPastDeadband(double rawX, double rawY, double deadband) {
        return isPastDeadband(Math.hypot(rawX, rawY), deadband);
    }

    public static double applySimpleDeadband(double raw, double deadband) {
        return isPastDeadband(raw, deadband) ? raw : 0;
    }

    public static Pair<Double, Double> applySimpleDeadband(double rawX, double rawY, double deadband) {
        double r = Math.hypot(rawX, rawY);
        double theta = Math.atan2(rawY, rawX);

        double newR = applySimpleDeadband(r, deadband);

        if (newR == 0) return Pair.of(0.0, 0.0);
        else return Pair.of(newR * Math.cos(theta), newR * Math.sin(theta));
    }

    public static double applyLinearDeadband(double raw, double deadband) {
        return isPastDeadband(raw, deadband) ? ((raw - deadband * Math.signum(raw)) / (1 - deadband)) : 0;
    }

    public static Pair<Double, Double> applyLinearDeadband(double rawX, double rawY, double deadband) {
        double r = Math.hypot(rawX, rawY);
        double theta = Math.atan2(rawY, rawX);

        double newR = applyLinearDeadband(r, deadband);

        if (newR == 0) return Pair.of(0.0, 0.0);
        else return Pair.of(newR * Math.cos(theta), newR * Math.sin(theta));
    }

    public static double applyExponentialDeadband(double raw, double deadband, int power) {
        // return Math.pow(Math.abs(applyLinearDeadband(raw, deadband)), power) * Math.signum(raw);
        return Math.copySign(Math.pow(applyLinearDeadband(raw, deadband), power), raw);
    }

    public static Pair<Double, Double> applyExponentialDeadband(double rawX, double rawY, double deadband, int power) {
        double r = Math.hypot(rawX, rawY);
        double theta = Math.atan2(rawY, rawX);

        double newR = applyExponentialDeadband(r, deadband, power);

        if (newR == 0) return Pair.of(0.0, 0.0);
        else return Pair.of(newR * Math.cos(theta), newR * Math.sin(theta));
    }

    public static double getOperatorSpaceJoystickAngle_rad(double x, double y) {
        return Math.atan2(-x, -y);
    }

    public static double getFieldSpaceJoystickAngle_rad(double x, double y) {
        return getOperatorSpaceJoystickAngle_rad(x, y) + (Robot.instance().brain.state.isRed ? Math.PI : 0);
    }
}
