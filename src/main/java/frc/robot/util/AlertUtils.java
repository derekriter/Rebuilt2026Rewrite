package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class AlertUtils {
    public static Alert makeDisconnectAlert(String deviceName, int can) {
        return new Alert(String.format("Missing connection to %s (CAN %d)", deviceName, can), AlertType.kError);
    }

    public static Alert makeOverheatingAlert(String deviceName, int can) {
        return new Alert(String.format("%s (CAN %d) is overheating", deviceName, can), AlertType.kWarning);
    }

    public static Alert makeCriticalOverheatingAlert(String deviceName, int can) {
        return new Alert(String.format("%s (CAN %d) is critically overheating", deviceName, can), AlertType.kError);
    }

    public static Alert makeConfigFailAlert(String deviceName, int can) {
        return new Alert(String.format("Failed to update %s (CAN %d) config", deviceName, can), AlertType.kError);
    }

    public static Alert makeHardwareFaultAlert(String deviceName, int can) {
        return new Alert(String.format("Hardware fault(s) active on %s (CAN %d)", deviceName, can), AlertType.kError);
    }

    public static Alert makeStallingAlert(String deviceName, int can) {
        return new Alert(String.format("Stalled detected on %s (CAN %d)", deviceName, can), AlertType.kWarning);
    }

    public static Alert makeSystemDisabledAlert(String systemName) {
        return new Alert(String.format("%s disabled by override"), AlertType.kInfo);
    }

    public static Alert makeSafetyDisabledAlert(String systemName) {
        return new Alert(String.format("%s safety disabled by override"), AlertType.kInfo);
    }
}
