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

    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    private final Alert breakerAlert = AlertUtils.makeBreakerTripAlert(LEDsConfig.systemName, LEDsConfig.channelID);

    private final SubsystemWriter<LEDs> subsystemWriter = Telemetry.makeSubsystemWriter(this, "/");
    private final BoolWriter breakerWriter;

    private boolean needsUpdate = false;

    public LEDs() {
        if (Overrides.disableLEDs) {
            AlertUtils.makeSystemDisabledAlert(LEDsConfig.systemName).set(true);

            leds = null;
            buffer = null;
            breakerWriter = null;
        } else {
            leds = new AddressableLED(LEDsConfig.dataPort);
            buffer = new AddressableLEDBuffer(LEDsConfig.ledCount);

            leds.setLength(buffer.getLength());
            leds.setColorOrder(LEDsConfig.colorOrder);

            leds.setData(buffer);
            leds.start();

            breakerWriter = Telemetry.makeBoolWriter(LEDsConfig.systemName, "breakerTripped");
        }
    }

    @Override
    public void periodic() {
        subsystemWriter.update();

        if (needsUpdate && leds != null) {
            leds.setData(buffer);
            needsUpdate = false;
        }
    }

    public void update() {
        if (leds == null) return;

        boolean breakerTripped = RobotContainer.instance().pdh.isBreakerTripped(LEDsConfig.channelID);

        breakerAlert.set(breakerTripped);
        breakerWriter.set(breakerTripped);
    }

    public void applyPattern(LEDPattern pattern) {
        if (leds == null) return;

        pattern.applyTo(buffer);
        needsUpdate = true;
    }

    public void clear() {
        applyPattern(LEDPattern.kOff);
    }
}
