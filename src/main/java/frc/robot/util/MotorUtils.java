package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.Supplier;

public class MotorUtils {
    /**
     * Safely apply a configuration to spark motor
     * @param motor
     * @param motorName Used for telemetry purposes
     * @param canID Used for telemetry purposes
     * @param channelID Used for telemetry purposes
     * @param config
     * @param resetMode
     * @param persistMode
     * @return {@code true} if the configuration was successfully applied, {@code false} otherwise
     */
    public static boolean safeApplyConfig(
            SparkBase motor,
            String motorName,
            int canID,
            int channelID,
            SparkBaseConfig config,
            ResetMode resetMode,
            PersistMode persistMode) {
        return safeApplyConfig(motor, motorName, canID, channelID, config, resetMode, persistMode, 3);
    }

    /**
     * Safely apply a configuration to spark motor
     * @param motor
     * @param motorName Used for telemetry purposes
     * @param canID Used for telemetry purposes
     * @param channelID Used for telemetry purposes
     * @param config
     * @param resetMode
     * @param persistMode
     * @param maxTries How many times to attempt to apply the configuration before failing
     * @return {@code true} if the configuration was successfully applied, {@code false} otherwise
     */
    public static boolean safeApplyConfig(
            SparkBase motor,
            String motorName,
            int canID,
            int channelID,
            SparkBaseConfig config,
            ResetMode resetMode,
            PersistMode persistMode,
            int maxTries) {
        if (motor == null) {
            Console.reportError("Cannot configure a null SparkBase", true);
            return false;
        }
        if (motorName == null || motorName.isEmpty()) {
            Console.reportWarning("Please provide a motor name when apply configurations", true);
            motorName = "UNNAMED SparkMotor";
        }
        if (config == null) {
            Console.reportError("Cannot apply a null SparkBaseConfig", true);
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
                Console.reportWarning(
                        String.format(
                                "Failed to configure %s (CAN %d, ch %d), retrying...", motorName, canID, channelID),
                        false);
            }
        }

        Console.reportError(String.format("Failed to configure %s (CAN %d, ch %d)", motorName, canID, channelID), true);
        AlertUtils.makeConfigFailAlert(motorName).set(true);
        return false;
    }

    public static boolean isSparkConnected(SparkBase spark) {
        return !spark.getFaults().firmware;
    }

    // Copyright (c) 2021-2026 Littleton Robotics
    // http://github.com/Mechanical-Advantage
    //
    // Use of this source code is governed by a BSD
    // license that can be found in the LICENSE_AdvantageKit file
    // at the root directory of this project.

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }
}
