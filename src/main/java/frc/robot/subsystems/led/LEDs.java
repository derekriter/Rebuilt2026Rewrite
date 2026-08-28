package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.LEDsConstants;
import frc.robot.constants.Overrides;
import frc.robot.util.AlertUtils;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {

    private final AddressableLED leds_nl;
    private final AddressableLEDBuffer buffer_nl;

    private final Alert breakerAlert = AlertUtils.makeBreakerTripAlert("LEDs");
    private boolean breakerLast = false;
    private boolean needsUpdate = false;

    public LEDs() {
        if (Overrides.disableLEDs) {
            AlertUtils.makeSystemDisabledAlert("LEDs").set(true);

            leds_nl = null;
            buffer_nl = null;
        } else {
            leds_nl = new AddressableLED(LEDsConstants.dataPort);
            buffer_nl = new AddressableLEDBuffer(LEDsConstants.ledCount);

            leds_nl.setLength(buffer_nl.getLength());
            leds_nl.setColorOrder(LEDsConstants.colorOrder);

            leds_nl.setData(buffer_nl);
            leds_nl.start();
        }
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        Logger.recordOutput("LEDs/currentCommand", currentCommand == null ? null : currentCommand.getName());

        Command defaultCommand = getDefaultCommand();
        Logger.recordOutput("LEDs/defaultCommand", defaultCommand == null ? null : defaultCommand.getName());

        if (leds_nl != null) {
            if (needsUpdate) {
                leds_nl.setData(buffer_nl);
                needsUpdate = false;
            }

            boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(LEDsConstants.channelID);

            Logger.recordOutput("LEDs/breakerTripped", breakerTripped);
            breakerAlert.set(breakerTripped);
            if (breakerTripped != breakerLast) {
                if (breakerTripped) {
                    Console.reportBreakerTripNoCAN("LEDs", LEDsConstants.channelID);
                } else {
                    Console.reportBreakerResetNoCAN("LEDs", LEDsConstants.channelID);
                }
            }

            breakerLast = breakerTripped;
        }
    }

    public void applyPattern(LEDPattern pattern) {
        if (leds_nl == null) return;

        pattern.applyTo(buffer_nl);
        needsUpdate = true;
    }

    public void clear() {
        applyPattern(LEDPattern.kOff);
    }
}
