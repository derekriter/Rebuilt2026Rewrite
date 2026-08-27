package frc.robot.subsystems.launcher.shooter;

import static frc.robot.config.LauncherConfig.ShooterConfig.*;

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
                "shooterMotor",
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
            inputs.pos_rots = shooter.getEncoder().getPosition(); // frame 2
            inputs.vel_RPM = shooter.getEncoder().getVelocity(); // frame 1
            inputs.temp_C = shooter.getMotorTemperature(); // frame 1
            inputs.appliedOut_perc = shooter.getAppliedOutput(); // frame 0
            inputs.voltageOut_V = shooter.getBusVoltage() * inputs.appliedOut_perc; // frame 0 & 1
            inputs.currentOut_A = shooter.getOutputCurrent(); // frame 1
        }
    }

    @Override
    public void setVelocityTarget(double velocity_rpm) {
        shooter.getClosedLoopController().setSetpoint(velocity_rpm, ControlType.kVelocity);
    }

    @Override
    public void setVoltage(double voltage_volts) {
        shooter.setVoltage(voltage_volts);
    }

    @Override
    public void stop() {
        shooter.stopMotor();
    }
}
