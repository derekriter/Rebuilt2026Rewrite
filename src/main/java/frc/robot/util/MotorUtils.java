package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import java.util.function.Supplier;

public class MotorUtils {

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

    public static boolean safeApplyConfig(
            SparkBase motor,
            String motorName,
            int canID,
            int channelID,
            SparkBaseConfig config,
            ResetMode resetMode,
            PersistMode persistMode,
            int maxTries) {
        if (motorName == null || motorName.isEmpty()) {
            Console.reportWarning("Please provide a motor name when apply configurations", true);
            motorName = "UNNAMED SparkMotor";
        }

        for (int i = 0; i < maxTries; i++) {
            try {
                REVLibError status = motor.configure(config, resetMode, persistMode);

                if (status == REVLibError.kOk) {
                    return true;
                }
            } catch (Throwable e) {
                Console.reportError(e, true);
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

    public static boolean safeApplyConfig(
            TalonFX motor, String motorName, int canID, int channelID, TalonFXConfiguration config) {
        return safeApplyConfig(motor, motorName, canID, channelID, config, 3);
    }

    public static boolean safeApplyConfig(
            TalonFX motor, String motorName, int canID, int channelID, TalonFXConfiguration config, int maxTries) {
        if (motorName == null || motorName.isEmpty()) {
            Console.reportWarning("Please provide a motor name when apply configurations", true);
            motorName = "UNNAMED TalonFX";
        }

        for (int i = 0; i < maxTries; i++) {
            try {
                StatusCode status = motor.getConfigurator().apply(config);

                if (status.isOK()) {
                    return true;
                }
            } catch (Throwable e) {
                Console.reportError(e, true);
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
