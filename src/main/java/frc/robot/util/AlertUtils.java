package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class AlertUtils {
    public static Alert makeCANFailureAlert(String deviceName) {
        return new Alert(String.format("Missing CAN connection to %s", deviceName), AlertType.kError);
    }

    public static Alert makeTempWarnAlert(String deviceName) {
        return new Alert(String.format("%s nearing thermal shutdown", deviceName), AlertType.kWarning);
    }

    public static Alert makeThermalShutdownAlert(String deviceName) {
        return new Alert(String.format("Thermal shutdown triggered on %s", deviceName), AlertType.kError);
    }

    public static Alert makeConfigFailAlert(String deviceName) {
        return new Alert(String.format("Failed to update %s config", deviceName), AlertType.kError);
    }

    public static Alert makeSystemDisabledAlert(String systemName) {
        return new Alert(String.format("%s disabled by override", systemName), AlertType.kWarning);
    }

    public static Alert makeSafetyDisabledAlert(String systemName) {
        return new Alert(String.format("%s safety disabled by override", systemName), AlertType.kWarning);
    }

    public static Alert makeBreakerTripAlert(String deviceName) {
        return new Alert(String.format("%s breaker tripped", deviceName), AlertType.kError);
    }
}
