package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.config.LEDsConfig;
import frc.robot.config.Overrides;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.compound.SubsystemWriter;
import frc.robot.util.AlertUtils;

public class LEDs extends SubsystemBase {

    private final AddressableLED leds_nl;
    private final AddressableLEDBuffer buffer_nl;

    private final Alert breakerAlert = AlertUtils.makeBreakerTripAlert(LEDsConfig.systemName);
    private boolean breakerLast = false;

    private final SubsystemWriter<LEDs> subsystemWriter =
            Telemetry.makeSubsystemWriter(this, "/", LEDsConfig.systemName);
    private final BoolWriter breakerWriter_nl;

    private boolean needsUpdate = false;

    public LEDs() {
        if (Overrides.disableLEDs) {
            AlertUtils.makeSystemDisabledAlert(LEDsConfig.systemName).set(true);

            leds_nl = null;
            buffer_nl = null;
            breakerWriter_nl = null;
        } else {
            leds_nl = new AddressableLED(LEDsConfig.dataPort);
            buffer_nl = new AddressableLEDBuffer(LEDsConfig.ledCount);

            leds_nl.setLength(buffer_nl.getLength());
            leds_nl.setColorOrder(LEDsConfig.colorOrder);

            leds_nl.setData(buffer_nl);
            leds_nl.start();

            breakerWriter_nl = Telemetry.makeBoolWriter(LEDsConfig.systemName, "breakerTripped");
        }
    }

    @Override
    public void periodic() {
        subsystemWriter.update();

        if (needsUpdate && leds_nl != null) {
            leds_nl.setData(buffer_nl);
            needsUpdate = false;
        }
    }

    public void update() {
        if (leds_nl == null) return;

        boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(LEDsConfig.channelID);

        breakerWriter_nl.set(breakerTripped);
        breakerAlert.set(breakerTripped);
        if (breakerTripped != breakerLast) {
            if (breakerTripped) {
                Telemetry.reportBreakerTripNoCAN(LEDsConfig.systemName, LEDsConfig.channelID);
            } else {
                Telemetry.reportBreakerResetNoCAN(LEDsConfig.systemName, LEDsConfig.channelID);
            }
        }

        breakerLast = breakerTripped;
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
