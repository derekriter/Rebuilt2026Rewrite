package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class AlertUtils {
    public static Alert makeDisconnectAlert(String deviceName, int can) {
        return new Alert(String.format("Missing connection to %s (CAN %d)", deviceName, can), AlertType.kError);
    }

    public static Alert makeTempWarnAlert(String deviceName, int can) {
        return new Alert(String.format("%s (CAN %d) nearing thermal shutdown", deviceName, can), AlertType.kWarning);
    }

    public static Alert makeThermalShutdownAlert(String deviceName, int can) {
        return new Alert(String.format("Thermal shutdown triggered on %s (CAN %d)", deviceName, can), AlertType.kError);
    }

    public static Alert makeConfigFailAlert(String deviceName, int can) {
        return new Alert(String.format("Failed to update %s (CAN %d) config", deviceName, can), AlertType.kError);
    }

    public static Alert makeSystemDisabledAlert(String systemName) {
        return new Alert(String.format("%s disabled by override", systemName), AlertType.kWarning);
    }

    public static Alert makeSafetyDisabledAlert(String systemName) {
        return new Alert(String.format("%s safety disabled by override", systemName), AlertType.kWarning);
    }
}
