package frc.robot.config;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDsConfig {
    public static final int dataPort = 9;
    public static final int channelID = 0; // TODO: LEDs channel id

    public static final int ledCount = 20 * 3;
    public static final Distance ledSpacing = Meters.of(1 / 20.0);
    public static final ColorOrder colorOrder = ColorOrder.kRGB;

    public static final Color chargeGreen = new Color("#008800");
    public static final Color chargeGold = new Color("#ffaa00");
    public static final Color orange = new Color("#ff2200");

    private LEDsConfig() {}
}
