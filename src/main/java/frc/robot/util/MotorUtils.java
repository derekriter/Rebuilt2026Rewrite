package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.telemetry.Telemetry;

public class MotorUtils {
    /**
     * Safely apply a configuration to spark motor
     * @param motor
     * @param motorName Used for logging purposes
     * @param config
     * @param resetMode
     * @param persistMode
     * @return {@code true} if the configuration was successfully applied, {@code false} otherwise
     */
    public static boolean safeApplyConfig(
            SparkBase motor, String motorName, SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
        return safeApplyConfig(motor, motorName, config, resetMode, persistMode, 3);
    }

    /**
     * Safely apply a configuration to spark motor
     * @param motor
     * @param motorName Used for logging purposes
     * @param config
     * @param resetMode
     * @param persistMode
     * @param maxTries How many times to attempt to apply the configuration before failing
     * @return {@code true} if the configuration was successfully applied, {@code false} otherwise
     */
    public static boolean safeApplyConfig(
            SparkBase motor,
            String motorName,
            SparkBaseConfig config,
            ResetMode resetMode,
            PersistMode persistMode,
            int maxTries) {
        if (motor == null) {
            Telemetry.reportError("Cannot configure a null SparkBase");
            return false;
        }
        if (motorName == null || motorName.isEmpty()) {
            Telemetry.reportWarning("Please provide a motor name when apply configurations", true);
            motorName = "UNNAMED SparkMotor";
        }
        if (config == null) {
            Telemetry.reportError("Cannot apply a null SparkBaseConfig");
            return false;
        }

        for (int i = 0; i < maxTries; i++) {
            try {
                REVLibError status = motor.configure(config, resetMode, persistMode);

                if (status == REVLibError.kOk) {
                    return true;
                }
            } catch (Throwable e) {
                e.printStackTrace();
            }

            if (i < maxTries - 1) {
                Telemetry.reportWarning(String.format("Failed to configure %s, retrying...", motorName), false);
            }
        }

        Telemetry.reportError(String.format("Failed to configure %s (CAN %d)", motorName, motor.getDeviceId()));
        AlertUtils.makeConfigFailAlert(motorName, motor.getDeviceId()).set(true);
        return false;
    }
}
