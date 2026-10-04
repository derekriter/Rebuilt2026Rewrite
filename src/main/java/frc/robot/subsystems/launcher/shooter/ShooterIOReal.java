package frc.robot.subsystems.launcher.shooter;

import static frc.robot.constants.LauncherConstants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.util.MotorUtils;

public class ShooterIOReal implements IShooterIO {

    private final SparkFlex shooter;

    public ShooterIOReal() {
        shooter = new SparkFlex(canID, MotorType.kBrushless);
        MotorUtils.safeApplyConfig(
                shooter,
                "shooter",
                canID,
                channelID,
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.connected = MotorUtils.isSparkConnected(shooter);

        if (inputs.connected) {
            inputs.vel_RPM = shooter.getEncoder().getVelocity();
            inputs.temp_C = shooter.getMotorTemperature();
            inputs.appliedOut_perc = shooter.getAppliedOutput();
            inputs.statorVoltage_V = shooter.getBusVoltage() * inputs.appliedOut_perc;
            inputs.statorCurrent_A = shooter.getOutputCurrent();
            inputs.thermalShutdown = MotorUtils.isSparkThermalShutdown(shooter);
        }
    }

    @Override
    public void setVelocityTarget(double velocity_RPM) {
        shooter.getClosedLoopController().setSetpoint(velocity_RPM, ControlType.kVelocity);
    }

    @Override
    public void setVoltage(double voltage_V) {
        shooter.setVoltage(voltage_V);
    }
}
